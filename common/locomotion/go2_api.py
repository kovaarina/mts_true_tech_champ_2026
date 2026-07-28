#!/usr/bin/env python3
"""go2_api — the participant-facing robot API for the Go2 obstacle-course competition.

YOUR controller is ONE Webots Python file. You create a `Go2`, then loop: read sensors,
decide, drive. The provided locomotion turns your velocity commands into robot motion (the
legs walk; balance is handled for you) — you write navigation/perception, NOT leg control.

    from go2_api import Go2
    bot = Go2()
    while bot.step():
        x, y, yaw = bot.pose()          # localization (GPS + IMU)
        ranges = bot.lidar()            # 2D scan for obstacles
        bot.drive(vx=0.4, vyaw=0.0)     # walk forward 0.4 m/s

Sensors:  pose() → (x, y, yaw) · lidar() → list of ranges · image() → front camera bytes
          · imu() → (roll, pitch, yaw) · time() → simulation seconds.
Postures: stand() · crouch(height) — e.g. crouch(0.18) to pass under a low bar.
Motion:   drive(vx, vyaw, vy=0) — vx fwd m/s (±0.8), vyaw turn rad/s (±1.5), vy strafe (±0.4).
          The robot REALLY walks (firmware policy, full physics): cruise ~0.4–0.5 m/s is
          reliable; aggressive speed/turns can make it fall.

ADVANCED (optional) — ROS 2:  `Go2(enable_ros2=True)` also runs a ROS 2 node that
publishes /scan (LaserScan), /odom (Odometry), /imu, /camera/image_raw and subscribes
/cmd_vel (Twist). Ignore the Python API and drive it from Nav2 / your own SLAM stack if
you prefer. If rclpy isn't present the bridge silently disables and the Python API still
works. (Locomotion, sensors and this file are provided/fixed — you only edit YOUR controller.)
"""
import math
import os
import sys

import struct

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller import Robot                       # noqa: E402  (Webots)
from policy_walk import PolicyWalk                 # noqa: E402  (firmware MNN locomotion)


class RemoteArm:
    """Client of the STATIONARY manipulator station (levels 2-3). The arm stands on the map
    next to its task and is driven over a radio link — fully IN PARALLEL with driving the
    robot: every call here is NON-BLOCKING; keep calling robot.step() and poll the state.

    Coordinates are in the ARM's own frame: origin at its pedestal centre on the floor,
    z up, axes aligned with the station. Reach ≈ 0.55 m around the pedestal.

        robot.arm.move(x, y, z)      send the gripper to a point (top-grasp, tip points down)
        robot.arm.at_target() -> bool  arrived at the last move() target (±1.5 cm)
        robot.arm.tip() -> (x,y,z)     current gripper position
        robot.arm.grab() / release()   electromagnet on / off (latches objects ≤ 9 cm away)
        robot.arm.holding() -> bool    an object is latched right now
        robot.arm.stow()               fold the arm
        robot.arm.request_image() / image()   wrist RGB:   ask for a frame / get the latest
                                              -> (w, h, bgra_bytes) or None
        robot.arm.request_depth() / depth()   wrist depth: -> (w, h, float32_bytes_m) or None
    """

    def __init__(self, robot, ts):
        self._tx = None
        self._rx = None
        try:
            self._tx = robot.getDevice("arm_tx")
            self._rx = robot.getDevice("arm_rx")
        except Exception:  # noqa: BLE001
            pass
        if self._rx:
            self._rx.enable(ts)
        self.ok = self._tx is not None and self._rx is not None
        self._tip = (0.0, 0.0, 0.0)
        self._holding = False
        self._at_target = False
        self._img = None
        self._dep = None
        self._cool = 0                 # ticks to ignore stale at_target after a new command

    def _poll(self):
        """Drain telemetry (called by Go2.step())."""
        if not self.ok:
            return
        if self._cool > 0:
            self._cool -= 1
        while self._rx.getQueueLength() > 0:
            data = self._rx.getBytes()
            self._rx.nextPacket()
            if not data:
                continue
            op = data[:1]
            if op == b'T' and len(data) >= 15:
                x, y, z, hold, at = struct.unpack("<3f2B", data[1:15])
                self._tip, self._holding, self._at_target = (x, y, z), bool(hold), bool(at)
            elif op == b'I' and len(data) >= 5:
                w, h = struct.unpack("<2H", data[1:5])
                self._img = (w, h, data[5:])
            elif op == b'D' and len(data) >= 5:
                w, h = struct.unpack("<2H", data[1:5])
                self._dep = (w, h, data[5:])

    def _send(self, payload):
        if self.ok:
            self._tx.send(payload)

    # ── commands (non-blocking) ────────────────────────────────────────────────
    def move(self, x, y, z):
        self._at_target = False
        self._cool = 30                # ~0.12 s: skip telemetry sent before the command landed
        self._send(b'M' + struct.pack("<3f", float(x), float(y), float(z)))

    def grab(self):
        self._send(b'G')

    def release(self):
        self._send(b'R')

    def stow(self):
        self._at_target = False
        self._cool = 30
        self._send(b'S')

    def request_image(self):
        self._send(b'I')

    def request_depth(self):
        self._send(b'D')

    # ── state (from the last telemetry packet, ~32 ms fresh) ──────────────────
    def at_target(self):
        return self._at_target and self._cool == 0

    def tip(self):
        return self._tip

    def holding(self):
        return self._holding

    def image(self):
        return self._img

    def depth(self):
        return self._dep


class Go2:
    # command caps (m/s, rad/s). The firmware policy is verified stable at ≤0.7 m/s; strafe
    # (vy) is supported too — see drive().
    MAX_VX = 0.8
    MAX_VY = 0.4
    MAX_VYAW = 1.5

    def __init__(self, enable_ros2=False, lidar=True, camera=True):
        self.robot = Robot()
        self.ts = int(self.robot.getBasicTimeStep())
        self.dt = self.ts / 1000.0

        # REAL locomotion: the Go2's own firmware walk policy (MNN) torque-drives the 12 leg
        # joints under full physics — the same verified stack as ai-robot/sim (go2_rl).
        self._walk = PolicyWalk(self.robot)

        # remote client of the stationary manipulator station (levels 2-3; ok=False elsewhere)
        self.arm = RemoteArm(self.robot, self.ts)

        self._gps = self._dev("gps")
        self._imu = self._dev("imu")
        self._lidar = self._dev("lidar") if lidar else None
        self._camera = self._dev("camera") if camera else None
        for d in (self._gps, self._imu, self._lidar, self._camera):
            if d is not None:
                d.enable(self.ts)

        self._bridge = _Ros2Bridge(self) if enable_ros2 else None

    def _dev(self, name):
        try:
            return self.robot.getDevice(name)
        except Exception:              # noqa: BLE001
            return None

    # ── commands ────────────────────────────────────────────────────────────────
    def drive(self, vx=0.0, vyaw=0.0, vy=0.0):
        """Command body velocity: vx forward (m/s), vyaw turn (rad/s, + = left),
        vy sideways strafe (m/s, + = left). The robot WALKS — expect real dynamics
        (inertia, small lag); it can fall if you slam it into walls at speed."""
        self._walk.command(
            max(-self.MAX_VX, min(self.MAX_VX, float(vx))),
            max(-self.MAX_VY, min(self.MAX_VY, float(vy))),
            max(-self.MAX_VYAW, min(self.MAX_VYAW, float(vyaw))),
        )

    def stop(self):
        self.drive(0.0, 0.0)

    def stand(self):
        """Hold a stable stand (policy paused, PD to the stand pose)."""
        self._walk.stand()

    def crouch(self, height=0.18):
        """Lower the body into a crouch (scripted pose)."""
        self._walk.crouch(height)

    # ── the tick: run the walk policy → advance physics → refresh sensors ────────
    def step(self):
        self._walk.tick()
        r = self.robot.step(self.ts)
        self.arm._poll()                       # drain manipulator telemetry (if present)
        if self._bridge is not None:
            self._bridge.publish()             # AFTER the step: sensors now hold this tick's data
            self._bridge.spin()
        return r != -1

    # ── sensors ─────────────────────────────────────────────────────────────────
    def pose(self):
        """(x, y, yaw) in world metres/rad — GPS position + IMU heading."""
        x, y, _ = self._gps.getValues() if self._gps else (0.0, 0.0, 0.0)
        yaw = self._imu.getRollPitchYaw()[2] if self._imu else 0.0
        return (x, y, yaw)

    def position(self):
        return tuple(self._gps.getValues()) if self._gps else (0.0, 0.0, 0.0)

    def imu(self):
        return tuple(self._imu.getRollPitchYaw()) if self._imu else (0.0, 0.0, 0.0)

    def lidar(self):
        """Full 3D LiDAR cloud — THIS is the scan to use for walls & navigation. Returns
        horizontalResolution × numberOfLayers ranges (m), layer by layer; `inf` = no return. Reshape
        with lidar_info() as [layer][azimuth]. It's a 3D dome, so walls are captured at any range
        regardless of sensor height or the body's gait pitch. Build an obstacle map by keeping, per
        azimuth, the nearest return whose 3D hit point is ABOVE the floor (that also naturally drops
        flat floor-markers such as checkpoints). The widest down/rear rays graze the robot's own body
        (a roughly constant near return ~0.2 m) — filter those. Ready-to-use snippet in API.md."""
        if not self._lidar:
            return []
        img = self._lidar.getRangeImage()      # None before the first robot.step()
        return list(img) if img else []

    def lidar_layer(self, layer=None):
        """One raw horizontal ring (m) from a SINGLE layer — optional/advanced. The default layer
        is near-horizontal at sensor height, so it can skim OVER low walls; for reliable wall
        detection use the full 3D cloud lidar() instead. Pass `layer` (0..layers-1) for a set ring."""
        if not self._lidar:
            return []
        if layer is None:
            layer = self._lidar.getNumberOfLayers() // 2
        img = self._lidar.getLayerRangeImage(layer)   # None before the first robot.step()
        return list(img) if img else []

    def lidar_info(self):
        """(h_fov_rad, h_res, max_range, layers, v_fov_rad). Reshape lidar() as [layer][h_res];
        bearing of horizontal index i ≈ -h_fov/2 + i*h_fov/(h_res-1)."""
        if not self._lidar:
            return (0.0, 0, 0.0, 0, 0.0)
        return (self._lidar.getFov(), self._lidar.getHorizontalResolution(),
                self._lidar.getMaxRange(), self._lidar.getNumberOfLayers(),
                self._lidar.getVerticalFov())

    def image(self):
        """Raw front-camera BGRA bytes (width×height×4) or None. See camera_info()."""
        return self._camera.getImage() if self._camera else None

    def camera_info(self):
        if not self._camera:
            return (0, 0, 0.0)
        return (self._camera.getWidth(), self._camera.getHeight(), self._camera.getFov())

    def time(self):
        return self.robot.getTime()


class _Ros2Bridge:
    """Optional ROS 2 bridge (guarded). Publishes /scan /odom /imu /camera/image_raw,
    subscribes /cmd_vel → drive(). Disables itself cleanly if rclpy isn't installed."""

    def __init__(self, bot: "Go2"):
        self.bot = bot
        self.ok = False
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import LaserScan, Imu
            from nav_msgs.msg import Odometry
            self._rclpy = rclpy
            self._LaserScan, self._Imu, self._Odometry = LaserScan, Imu, Odometry
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node("go2_participant")
            self.node.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
            self.pub_scan = self.node.create_publisher(LaserScan, "scan", 10)
            self.pub_odom = self.node.create_publisher(Odometry, "odom", 10)
            self.pub_imu = self.node.create_publisher(Imu, "imu", 10)
            self.ok = True
        except Exception as e:  # noqa: BLE001 — no ROS2 → Python API still works
            print(f"[go2_api] ROS2 bridge disabled ({e}); use the Python API.", flush=True)

    def _on_cmd(self, msg):
        self.bot.drive(msg.linear.x, msg.angular.z)

    def spin(self):
        if self.ok:
            self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish(self):
        if not self.ok:
            return
        now = self.node.get_clock().now().to_msg()
        # /scan (LaserScan is 2D → publish the middle horizontal layer of the 3D lidar)
        ranges = self.bot.lidar_layer()
        if ranges:
            info = self.bot.lidar_info()
            fov, n, mx = info[0], info[1], info[2]
            s = self._LaserScan()
            s.header.stamp = now
            s.header.frame_id = "base_link"
            s.angle_min = -fov / 2.0
            s.angle_max = fov / 2.0
            s.angle_increment = fov / max(1, n - 1)
            s.range_min = 0.05
            s.range_max = float(mx)
            s.ranges = [float(r) for r in ranges]
            self.pub_scan.publish(s)
        # /odom (pose from GPS+IMU; no covariance — it's ground truth-ish)
        x, y, yaw = self.bot.pose()
        od = self._Odometry()
        od.header.stamp = now
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
        od.pose.pose.orientation.z = math.sin(yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_odom.publish(od)
        # /imu (orientation only)
        r, p, yw = self.bot.imu()
        im = self._Imu()
        im.header.stamp = now
        im.header.frame_id = "base_link"
        cy, sy = math.cos(yw / 2), math.sin(yw / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        im.orientation.w = cr * cp * cy + sr * sp * sy
        im.orientation.x = sr * cp * cy - cr * sp * sy
        im.orientation.y = cr * sp * cy + sr * cp * sy
        im.orientation.z = cr * cp * sy - sr * sp * cy
        self.pub_imu.publish(im)
