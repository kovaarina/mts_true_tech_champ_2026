#!/usr/bin/env python3
"""Autonomous controller for level 1.

The course files are deliberately not used.  Navigation is based only on the
on-board camera, 3-D lidar, IMU and pose supplied by ``go2_api``.
"""
import math
import os
import sys
import heapq
from collections import deque

import numpy as np


# Fixed locomotion package supplied by the organisers.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "common", "locomotion"))
from go2_api import Go2  # noqa: E402


INF = float("inf")


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Navigator:
    """Marker tracking plus a small vector-field local planner."""

    def __init__(self, robot):
        self.robot = robot
        self.fov, self.scan_size, self.max_range, self.layers, self.v_fov = robot.lidar_info()
        self.cam_w, self.cam_h, self.cam_fov = robot.camera_info()

        self.checkpoints = 0
        self.target = None                 # filtered world (x, y) of the current marker
        self.target_samples = deque(maxlen=7)
        self.last_target = None
        self.last_seen = -100.0
        self.waiting_for_next = False
        self.reached_at = -100.0

        self.turn_memory = 0.0
        self.turn_lock_until = -1.0
        self.turn_start_yaw = 0.0
        self.vx_cmd = 0.0
        self.wz_cmd = 0.0
        self.map_resolution = 0.16
        self.wall_cells = set()
        self.blocked_cells = set()
        self.free_cells = set()
        self.visit_counts = {}
        self.occupied = self.blocked_cells
        self.path = []
        self.path_goal = None
        self.next_plan_time = 0.0
        self.next_map_save_time = 0.0
        self.last_visit_cell = None
        self.wall_distance_cache = {}
        self.map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "lidar_map.npz")

    # ---- perception -----------------------------------------------------
    def obstacle_scan(self):
        """Nearest non-floor 3-D lidar return for every azimuth."""
        cloud = self.robot.lidar()
        if not cloud or self.scan_size <= 0 or self.layers <= 0:
            return np.full(max(1, self.scan_size), self.max_range or 12.0)

        ranges = np.asarray(cloud, dtype=np.float64).reshape(self.layers, self.scan_size)
        layer_angles = np.linspace(self.v_fov / 2.0, -self.v_fov / 2.0,
                                   self.layers)[:, None]
        lidar_z = self.robot.position()[2] + 0.16
        hit_z = lidar_z + ranges * np.sin(layer_angles)
        azimuth = np.linspace(-self.fov / 2.0, self.fov / 2.0,
                              self.scan_size)[None, :]
        near_rear_self = ((ranges < 0.45) &
                          (np.abs(azimuth) > math.radians(95.0)))
        valid = (np.isfinite(ranges) & (ranges < self.max_range) &
                 (ranges >= 0.30) & ~near_rear_self & (hit_z >= 0.15))
        filtered = np.where(valid, ranges, np.inf)
        scan = np.min(filtered, axis=0)
        return np.where(np.isfinite(scan), scan, self.max_range)

    def marker_measurement(self, kind):
        """Return (pixel count, centroid x, centroid y, world point) or None."""
        raw = self.robot.image()
        if raw is None or self.cam_w <= 0:
            return None
        image = np.frombuffer(raw, dtype=np.uint8).reshape(self.cam_h, self.cam_w, 4)
        b = image[:, :, 0].astype(np.int16)
        g = image[:, :, 1].astype(np.int16)
        r = image[:, :, 2].astype(np.int16)

        # Ratios tolerate Webots lighting while rejecting grey walls and floor.
        if kind == "yellow":
            mask = ((r > 190) & (g > 190) & (b > 95) &
                    (b * 4 < np.minimum(r, g) * 3) &
                    (np.abs(r - g) < 70))
        else:  # purple finish
            mask = ((r > 55) & (b > 55) & (g * 5 < np.maximum(r, b) * 4) &
                    (np.minimum(r, b) * 2 > np.maximum(r, b)))

        # Markers are on the floor.  Dropping the top strip also rejects UI/sky artefacts.
        mask[:self.cam_h // 5, :] = False
        ys, xs = np.nonzero(mask)
        if xs.size < 300:
            return None

        # A trimmed centroid is steadier at the anti-aliased edge of a marker.
        cx = float(np.median(xs))
        cy = float(np.median(ys))
        point = self.pixel_to_ground(cx, cy)
        return int(xs.size), cx, cy, point

    def pixel_to_ground(self, u, v):
        """Project an image pixel onto z=0 using the measured body attitude."""
        focal = self.cam_w / (2.0 * math.tan(self.cam_fov / 2.0))
        ray = np.array([1.0, -(u - (self.cam_w - 1) / 2.0) / focal,
                        -(v - (self.cam_h - 1) / 2.0) / focal])
        roll, pitch, yaw = self.robot.imu()
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        rotation = rz @ ry @ rx
        x, y, z = self.robot.position()
        origin = np.array([x, y, z]) + rotation @ np.array([0.40, 0.0, 0.07])
        world_ray = rotation @ ray
        if world_ray[2] >= -0.025:
            return None
        distance = -origin[2] / world_ray[2]
        if distance <= 0.0 or distance > 10.0:
            return None
        hit = origin + distance * world_ray
        return float(hit[0]), float(hit[1])

    # ---- target state ---------------------------------------------------
    def set_measurement(self, measurement, now, x, y):
        if measurement is None:
            return
        _, _, _, point = measurement
        self.last_seen = now
        if point is None:
            return

        # Once a marker is reached, reject its last stale camera frame.  A genuinely
        # new yellow marker is spatially separated and is accepted immediately.
        if self.waiting_for_next and self.last_target is not None:
            if math.dist(point, self.last_target) < 0.65 and now - self.reached_at < 1.2:
                return
            self.waiting_for_next = False
            self.target_samples.clear()

        # A sudden marker jump while standing at the old target means the referee has
        # recoloured the old checkpoint and highlighted the next one.
        if self.target is not None and math.dist(point, self.target) > 0.9:
            if math.hypot(x - self.target[0], y - self.target[1]) < 0.65:
                self.mark_checkpoint_reached(now)
                self.waiting_for_next = False
                self.target_samples.clear()
            else:
                return

        self.target_samples.append(point)
        points = np.asarray(self.target_samples)
        self.target = (float(np.median(points[:, 0])), float(np.median(points[:, 1])))

    def mark_checkpoint_reached(self, now):
        if self.checkpoints >= 4:
            return
        self.checkpoints += 1
        self.last_target = self.target
        self.target = None
        self.target_samples.clear()
        self.waiting_for_next = True
        self.reached_at = now

    # ---- local planning -------------------------------------------------
    def choose_heading(self, scan, desired):
        bearings = np.linspace(-self.fov / 2.0, self.fov / 2.0,
                               self.scan_size, endpoint=True)
        rr = np.asarray(scan)
        good = np.isfinite(rr) & (rr < self.max_range)
        ox = rr[good] * np.cos(bearings[good])
        oy = rr[good] * np.sin(bearings[good])

        front = self.sector_distance(scan, 0.0, math.radians(16.0))
        now = self.robot.time()
        locked = now < self.turn_lock_until
        yaw = self.robot.pose()[2]
        turn_progress = (wrap(yaw - self.turn_start_yaw) *
                         (1.0 if self.turn_memory >= 0.0 else -1.0))
        must_turn = locked and turn_progress < 1.0
        planning_desired = 0.0 if locked and not must_turn else desired

        candidates = np.radians(np.arange(-110.0, 111.0, 5.0))
        best_angle, best_score, best_clearance = 0.0, -1e9, 0.0
        for angle in candidates:
            ca, sa = math.cos(angle), math.sin(angle)
            along = ox * ca + oy * sa
            lateral = -ox * sa + oy * ca
            in_lane = (along > 0.0) & (np.abs(lateral) < 0.38)
            clearance = float(np.min(along[in_lane])) if np.any(in_lane) else self.max_range
            clearance = clamp(clearance, 0.0, 2.5)
            error = abs(wrap(angle - planning_desired))
            score = (1.08 * clearance - 1.30 * error + 0.12 * math.cos(angle) +
                     0.16 * math.cos(angle - self.turn_memory))
            if clearance < 0.38:
                score -= 4.0
            if score > best_score:
                best_angle, best_score, best_clearance = angle, score, clearance

        # Hysteresis prevents left/right chatter while negotiating a partition.
        if front < 0.95 or must_turn:
            if not locked:
                left = self.sector_distance(scan, math.radians(58.0), math.radians(24.0))
                right = self.sector_distance(scan, math.radians(-58.0), math.radians(24.0))
                if abs(left - right) > 0.08:
                    sign = 1.0 if left > right else -1.0
                elif abs(desired) > 0.12:
                    sign = math.copysign(1.0, desired)
                else:
                    sign = 1.0
                self.turn_memory = sign
                self.turn_start_yaw = yaw
                self.turn_lock_until = now + 13.0
            sign = math.copysign(1.0, self.turn_memory)
            best_angle = sign * max(0.90, abs(best_angle))
        elif not locked and front > 1.35 and abs(desired) < 0.4:
            self.turn_memory *= 0.90
        if best_clearance < 0.30 and abs(desired) > 0.15:
            best_angle = clamp(desired, -1.20, 1.20)
        return best_angle, best_clearance, front

    def sector_distance(self, scan, centre, half_width):
        bearings = np.linspace(-self.fov / 2.0, self.fov / 2.0,
                               self.scan_size, endpoint=True)
        delta = np.abs((bearings - centre + math.pi) % (2.0 * math.pi) - math.pi)
        values = np.asarray(scan)[delta <= half_width]
        if values.size == 0:
            return self.max_range
        # A low percentile is robust to an isolated gait/self reflection.
        return float(np.percentile(values, 12.0))

    # ---- small lidar map and global planner -----------------------------
    def grid_cell(self, x, y):
        return (int(round(x / self.map_resolution)),
                int(round(y / self.map_resolution)))

    def trace_cells(self, start, end):
        """Integer cells touched by a lidar ray, excluding the endpoint."""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        cells = []
        x, y = x0, y0
        while (x, y) != (x1, y1):
            cells.append((x, y))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def update_map(self, scan, x, y, yaw):
        bearings = np.linspace(-self.fov / 2.0, self.fov / 2.0,
                               self.scan_size, endpoint=True)
        rr = np.asarray(scan)
        robot_cell = self.grid_cell(x, y)
        self.free_cells.add(robot_cell)
        if robot_cell != self.last_visit_cell:
            self.visit_counts[robot_cell] = self.visit_counts.get(robot_cell, 0) + 1
            self.last_visit_cell = robot_cell

        valid_ray = np.isfinite(rr) & (rr > 0.42)
        ray_step = 6
        for distance, bearing in zip(rr[valid_ray][::ray_step], bearings[valid_ray][::ray_step]):
            clipped = min(float(distance), self.max_range - 0.20)
            if clipped <= 0.45:
                continue
            end = self.grid_cell(x + clipped * math.cos(yaw + float(bearing)),
                                 y + clipped * math.sin(yaw + float(bearing)))
            for cell in self.trace_cells(robot_cell, end)[:80]:
                if cell not in self.blocked_cells:
                    self.free_cells.add(cell)

        valid_hit = np.isfinite(rr) & (rr < self.max_range - 0.05) & (rr > 0.42)
        wx = x + rr[valid_hit] * np.cos(yaw + bearings[valid_hit])
        wy = y + rr[valid_hit] * np.sin(yaw + bearings[valid_hit])
        # Inflate by the body half-width plus a small gait margin.
        inflation = 3
        new_wall = False
        for px, py in zip(wx, wy):
            cell = self.grid_cell(float(px), float(py))
            if cell not in self.wall_cells:
                new_wall = True
            self.wall_cells.add(cell)
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    if dx * dx + dy * dy <= inflation * inflation + 1:
                        blocked = (cell[0] + dx, cell[1] + dy)
                        self.blocked_cells.add(blocked)
                        self.free_cells.discard(blocked)
        if new_wall:
            self.wall_distance_cache.clear()

    def distance_to_wall_cells(self, cell, max_radius=8):
        cached = self.wall_distance_cache.get(cell)
        if cached is not None:
            return cached
        if not self.wall_cells:
            return max_radius + 1
        best = max_radius + 1
        cx, cy = cell
        for dx in range(-max_radius, max_radius + 1):
            for dy in range(-max_radius, max_radius + 1):
                if dx * dx + dy * dy >= best * best:
                    continue
                if (cx + dx, cy + dy) in self.wall_cells:
                    best = max(1, int(round(math.hypot(dx, dy))))
        self.wall_distance_cache[cell] = best
        return best

    def traversal_cost(self, cell, step_cost):
        wall_distance = self.distance_to_wall_cells(cell)
        # Keep the route wall-aware: not scraping the wall, but still using walls
        # as corridor boundaries instead of drifting through unknown space.
        if wall_distance <= 2:
            wall_cost = 4.0
        elif wall_distance <= 4:
            wall_cost = 0.0
        elif wall_distance <= 7:
            wall_cost = 0.35 * (wall_distance - 4)
        else:
            wall_cost = 1.6

        known_cost = 0.0 if cell in self.free_cells else 0.75
        visit_cost = min(3.5, 0.10 * self.visit_counts.get(cell, 0))
        return step_cost + wall_cost + known_cost + visit_cost

    def build_path(self, start_xy, goal_xy):
        start = self.grid_cell(*start_xy)
        goal = self.grid_cell(*goal_xy)
        margin = int(3.8 / self.map_resolution)
        lo_x = min(start[0], goal[0]) - margin
        hi_x = max(start[0], goal[0]) + margin
        lo_y = min(start[1], goal[1]) - margin
        hi_y = max(start[1], goal[1]) + margin
        blocked = self.blocked_cells
        # The robot and marker cells must remain legal even next to an inflated wall.
        allowed = {(start[0] + dx, start[1] + dy)
                   for dx in range(-2, 3) for dy in range(-2, 3)
                   if dx * dx + dy * dy <= 4}
        allowed.update({(goal[0] + dx, goal[1] + dy)
                        for dx in range(-1, 2) for dy in range(-1, 2)})
        frontier = [(0.0, start)]
        cost = {start: 0.0}
        parent = {}
        moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        found = False
        for _ in range(30000):
            if not frontier:
                break
            _, current = heapq.heappop(frontier)
            if current == goal:
                found = True
                break
            base = cost[current]
            for dx, dy, step_cost in moves:
                nxt = (current[0] + dx, current[1] + dy)
                if not (lo_x <= nxt[0] <= hi_x and lo_y <= nxt[1] <= hi_y):
                    continue
                if nxt in blocked and nxt not in allowed:
                    continue
                # Do not cut diagonally through an occupied corner.
                if dx and dy and (((current[0] + dx, current[1]) in blocked) or
                                  ((current[0], current[1] + dy) in blocked)):
                    continue
                new_cost = base + self.traversal_cost(nxt, step_cost)
                if new_cost >= cost.get(nxt, INF):
                    continue
                cost[nxt] = new_cost
                parent[nxt] = current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
        if not found:
            return []
        cells = [goal]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        cells.reverse()
        return [(cx * self.map_resolution, cy * self.map_resolution) for cx, cy in cells]

    def save_lidar_map(self, now):
        if now < self.next_map_save_time:
            return
        self.next_map_save_time = now + 2.0
        try:
            visited = np.asarray([(x, y, c) for (x, y), c in self.visit_counts.items()],
                                 dtype=np.int16)
            path = np.asarray(self.path, dtype=np.float32)
            np.savez_compressed(
                self.map_path,
                resolution=np.asarray([self.map_resolution], dtype=np.float32),
                walls=np.asarray(list(self.wall_cells), dtype=np.int16),
                blocked=np.asarray(list(self.blocked_cells), dtype=np.int16),
                free=np.asarray(list(self.free_cells), dtype=np.int16),
                visited=visited,
                path=path,
            )
        except OSError:
            pass

    def planned_heading(self, x, y, yaw, now):
        if self.target is None:
            return None
        goal_changed = (self.path_goal is None or
                        math.dist(self.path_goal, self.target) > 0.35)
        if goal_changed or now >= self.next_plan_time or not self.path:
            self.path = self.build_path((x, y), self.target)
            self.path_goal = self.target
            self.next_plan_time = now + 0.45
        if not self.path:
            return None
        # Drop passed cells, then aim far enough ahead for smooth quadruped motion.
        while len(self.path) > 2 and math.hypot(self.path[1][0] - x,
                                                self.path[1][1] - y) < 0.30:
            self.path.pop(0)
        waypoint = self.path[-1]
        for point in self.path[1:]:
            waypoint = point
            if math.hypot(point[0] - x, point[1] - y) >= 0.70:
                break
        return wrap(math.atan2(waypoint[1] - y, waypoint[0] - x) - yaw)

    def smooth_drive(self, vx, wz):
        dt = max(0.016, float(getattr(self.robot, "dt", 0.032)))
        self.vx_cmd += clamp(vx - self.vx_cmd, -0.75 * dt, 0.75 * dt)
        self.wz_cmd += clamp(wz - self.wz_cmd, -3.2 * dt, 3.2 * dt)
        self.robot.drive(vx=self.vx_cmd, vyaw=self.wz_cmd)

    def tick(self):
        now = self.robot.time()
        x, y, yaw = self.robot.pose()

        kind = "purple" if self.checkpoints >= 4 else "yellow"
        measurement = self.marker_measurement(kind)
        self.set_measurement(measurement, now, x, y)

        distance = INF
        desired = 0.0
        if self.target is not None:
            distance = math.hypot(self.target[0] - x, self.target[1] - y)
            desired = wrap(math.atan2(self.target[1] - y, self.target[0] - x) - yaw)

        # Entering the marker centre is enough to make checkpoint scoring deterministic,
        # even when the camera lost the floor marker about half a metre earlier.
        reached_marker = (distance < 0.24 or
                          (measurement is None and distance < 0.48))
        if kind == "yellow" and self.target is not None and reached_marker:
            self.mark_checkpoint_reached(now)
            distance, desired = INF, 0.0

        # At the finish, stand rather than continuously correcting inside the small ring.
        if kind == "purple" and self.target is not None and distance < 0.16:
            self.vx_cmd = 0.0
            self.wz_cmd = 0.0
            self.robot.stand()
            return

        scan = self.obstacle_scan()
        self.update_map(scan, x, y, yaw)
        map_heading = self.planned_heading(x, y, yaw, now)
        self.save_lidar_map(now)
        if map_heading is None:
            heading, clearance, front = self.choose_heading(scan, desired)
        else:
            # A* already includes the inflated robot footprint.  Use the lidar once
            # more as an emergency short-range guard, not as the route selector.
            heading = clamp(map_heading, -1.50, 1.50)
            clearance = self.sector_distance(scan, heading, math.radians(18.0))
            front = self.sector_distance(scan, 0.0, math.radians(16.0))
            self.turn_lock_until = -1.0
            self.turn_memory = 0.0

        turn = clamp(1.7 * heading, -1.15, 1.15)
        a = abs(heading)
        if front < 0.43 or a > 1.05:
            speed = 0.035
        elif a > 0.65:
            speed = 0.14
        elif a > 0.32:
            speed = 0.29
        else:
            speed = 0.38
        if clearance < 0.30:
            speed = 0.0
        speed *= clamp((front - 0.28) / 0.65, 0.18, 1.0)
        speed *= clamp((clearance - 0.25) / 0.60, 0.25, 1.0)
        if distance < 0.75:
            speed = min(speed, 0.28)
        if distance < 0.38:
            speed = min(speed, 0.14)
        self.smooth_drive(speed, turn)


def main():
    robot = Go2()
    navigator = None
    while robot.step():
        if navigator is None:
            navigator = Navigator(robot)
        navigator.tick()


if __name__ == "__main__":
    main()
