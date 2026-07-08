#!/usr/bin/env python3
"""referee — the competition judge (Webots Supervisor), shared by all 3 levels.

Runs in its OWN Robot node (supervisor TRUE) in a SEPARATE controller from the
participant's Go2, so the participant can never touch the Supervisor API (no teleport,
no self-scoring). It only READS the robot's pose via the Supervisor API and drives all
scoring from position — the two sides talk through the world, not through code.

Config-driven: reads a level JSON (path from controllerArgs[0] or $REFEREE_CONFIG, else
`../config/level.json`). Scores sequential checkpoints, requires a hold in the finish
ring, ends on fall/flip or timeout, recolours checkpoints green as they score, writes
`result.json`, and (optionally) records the run video. Judging is by SIMULATION time.

result.json: { level, reason(finish|fall|timeout), score, base_score, bonus,
               checkpoints:[{name,scored,sim_time}], sim_time, wall_time, penalties }
"""
import json
import math
import os
import sys
import time

from controller import Supervisor

GREEN = [0.1, 0.9, 0.2]
YELLOW = [0.95, 0.85, 0.1]


def _load_config():
    path = None
    if len(sys.argv) > 1:
        path = sys.argv[1]
    path = os.environ.get("REFEREE_CONFIG", path)
    if not path:
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, "..", "config", "level.json")
    with open(path) as f:
        return json.load(f), path


class Referee:
    def __init__(self):
        self.sup = Supervisor()
        self.ts = int(self.sup.getBasicTimeStep())
        self.cfg, self.cfg_path = _load_config()

        self.robot = self.sup.getFromDef(self.cfg.get("robot_def", "ROBOT"))
        if self.robot is None:
            print("[referee] FATAL: robot DEF not found", flush=True)
        # The competition robot (Go2Base) is a kinematic base: the ROBOT node is a STATIC anchor and
        # the real body moves via joints as a PROTO-internal DEF (default "BODY"). We read that node's
        # WORLD pose with getPosition()/getOrientation(). Falls back to the robot node itself for a
        # plain (non-base) robot; set "body_def":"" in config to force the robot node.
        self._pose_node = None
        if self.robot is not None:
            body_def = self.cfg.get("body_def", "BODY")
            if body_def:
                try:
                    self._pose_node = self.robot.getFromProtoDef(body_def)
                except Exception:  # noqa: BLE001 — older API / not a proto
                    self._pose_node = None
            if self._pose_node is None:
                self._pose_node = self.robot
        print(f"[referee] pose source: {'BODY' if self._pose_node is not self.robot else 'ROBOT node'}",
              flush=True)

        self.cps = [dict(c, scored=False, sim_time=None) for c in self.cfg["checkpoints"]]
        # "sequential" (default): checkpoints must be taken in listed order, finish arms after
        # the last one. "any": checkpoints score in ANY order (scattered collectibles — they
        # deliberately do NOT trace the route) and the finish is armed from the start; the
        # completion bonus still requires collecting them all.
        self.order = self.cfg.get("checkpoint_order", "sequential")
        self.finish = self.cfg["finish"]
        self.bonus_pct = self.cfg.get("scoring", {}).get("completion_bonus_pct", 30)
        self.fall = self.cfg.get("fall", {"z_min": 0.12, "tilt_max_deg": 60})
        self.tmo = self.cfg.get("timeouts", {"sim_time_s": 240, "wall_clock_s": 900})
        # Optional: REFEREE_RESULT / REFEREE_VIDEO env vars override the config output paths
        # when set (used by the automated judge); otherwise paths are resolved from the config.
        self.result_path = os.environ.get("REFEREE_RESULT") or self.cfg.get("result_path", "result.json")
        self.video_path = os.environ.get("REFEREE_VIDEO") or self.cfg.get("video_path", "run.mp4")

        self._idx = 0                      # next checkpoint to score (sequential)
        self._finish_hold = 0.0            # s held inside the finish ring
        self._penalties = []
        self._wall_start = time.time()
        self._done = False

        # ── manipulation tasks: put OBJECT (by DEF) inside a zone → points, once. When ALL
        # tasks are done, the DOOR node slides under the floor (opens a shortcut/gate).
        # config: "manipulation": { "tasks": [{name, object_def, x, y, radius, z_min, z_max,
        # points}...], "door_def": "DOOR", "door_drop": 0.6 }
        man = self.cfg.get("manipulation", {})
        self.man_tasks = [dict(t, done=False, sim_time=None) for t in man.get("tasks", [])]
        self._man_nodes = {t["object_def"]: self.sup.getFromDef(t["object_def"])
                           for t in self.man_tasks}
        self._door = self.sup.getFromDef(man.get("door_def", "")) if man.get("door_def") else None
        self._door_drop = float(man.get("door_drop", 0.6))
        self._door_z = None                # opening animation state
        if self.man_tasks:
            missing = [d for d, n in self._man_nodes.items() if n is None]
            if missing or (man.get("door_def") and self._door is None):
                print(f"[referee] WARNING: manipulation DEFs missing: {missing} "
                      f"door={self._door is not None}", flush=True)

        self._recording = False
        # record if the config asks, or REFEREE_RECORD=1 is set in the environment
        want_video = self.cfg.get("record_video") or os.environ.get("REFEREE_RECORD") == "1"
        if want_video:
            fn = self.video_path
            try:
                self.sup.movieStartRecording(fn, width=1280, height=720, quality=70,
                                             codec=0, acceleration=1, caption=False)
                self._recording = True
            except Exception as e:  # noqa: BLE001
                print(f"[referee] video disabled: {e}", flush=True)
        print(f"[referee] level {self.cfg.get('level')} — {len(self.cps)} checkpoints, "
              f"config={os.path.basename(self.cfg_path)}", flush=True)

    # ── geometry ────────────────────────────────────────────────────────────────
    def _pose(self):
        p = self._pose_node.getPosition()          # world [x, y, z] of the moving body
        o = self._pose_node.getOrientation()       # row-major 3x3
        up_z = o[8]                                 # world-z component of the body up axis
        yaw = math.atan2(o[3], o[0])
        return p[0], p[1], p[2], up_z, yaw

    def _recolor(self, cp, color):
        node = self.sup.getFromDef(cp.get("def", ""))
        if node is not None:
            f = node.getField("color")
            if f is not None:
                f.setSFColor(color)

    # ── scoring ─────────────────────────────────────────────────────────────────
    def _base_score(self):
        return (sum(c["points"] for c in self.cps if c["scored"])
                + sum(t["points"] for t in self.man_tasks if t["done"]))

    def _final(self, reason):
        base = self._base_score()
        completed = (all(c["scored"] for c in self.cps)
                     and all(t["done"] for t in self.man_tasks)
                     and reason == "finish")
        bonus = round(base * self.bonus_pct / 100.0) if completed else 0
        score = base + bonus - sum(p.get("cost", 0) for p in self._penalties)
        result = {
            "level": self.cfg.get("level"),
            "reason": reason,
            "score": score,
            "base_score": base,
            "bonus": bonus,
            "completed": completed,
            "checkpoints": [{"name": c["name"], "scored": c["scored"], "sim_time": c["sim_time"]}
                            for c in self.cps],
            "sim_time": round(self.sup.getTime(), 3),
            "wall_time": round(time.time() - self._wall_start, 3),
            "penalties": self._penalties,
            "manipulation": [{"name": t["name"], "done": t["done"], "sim_time": t["sim_time"]}
                             for t in self.man_tasks],
        }
        try:
            with open(self.result_path, "w") as f:
                json.dump(result, f, indent=2)
        except Exception as e:  # noqa: BLE001
            print(f"[referee] result write failed: {e}", flush=True)
        print(f"[referee] DONE reason={reason} score={score} (base {base} + bonus {bonus}) "
              f"sim={result['sim_time']}s", flush=True)
        if self._recording:
            self.sup.movieStopRecording()
            # BLOCK until the async encoder finishes — quitting Webots earlier kills the encoder
            # and drops the file. Long runs (2+ min of 720p) can take minutes to encode; guard
            # with a generous wall-clock cap so a hung encoder can't wedge the judge forever.
            t0 = time.time()
            while not self.sup.movieIsReady() and time.time() - t0 < 900:
                self.sup.step(self.ts)
            print(f"[referee] video finalised (encode {time.time() - t0:.0f}s, "
                  f"ready={self.sup.movieIsReady()})", flush=True)
        self._done = True
        self.sup.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        self.sup.simulationQuit(0)

    # ── main loop ─────────────────────────────────────────────────────────────
    def run(self):
        if self.robot is None:
            self._final("error")
            return
        # Optional one-shot overview screenshot (demo/docs). REFEREE_SCREENSHOT=<path>.png →
        # render a few frames from the world Viewpoint, export, and quit (no scoring). Judging
        # runs never set this, so it has zero effect on results.
        shot = os.environ.get("REFEREE_SCREENSHOT")
        if shot:
            for _ in range(40):
                self.sup.step(self.ts)
            self.sup.exportImage(shot, 100)
            for _ in range(10):
                self.sup.step(self.ts)
            print(f"[referee] screenshot -> {shot}", flush=True)
            self.sup.simulationQuit(0)
            return
        while self.sup.step(self.ts) != -1 and not self._done:
            x, y, z, up_z, _ = self._pose()
            t = self.sup.getTime()

            # fall / flip
            if z < self.fall["z_min"] or up_z < math.cos(math.radians(self.fall["tilt_max_deg"])):
                self._penalties.append({"type": "fall", "sim_time": round(t, 2), "cost": 0})
                self._final("fall")
                return

            # timeouts
            if t > self.tmo["sim_time_s"]:
                self._final("timeout")
                return
            if time.time() - self._wall_start > self.tmo["wall_clock_s"]:
                self._penalties.append({"type": "wall_clock_hang", "cost": 0})
                self._final("timeout")
                return

            # manipulation tasks: object inside its zone → done (latched), points
            if self.man_tasks:
                for task in self.man_tasks:
                    if task["done"]:
                        continue
                    node = self._man_nodes.get(task["object_def"])
                    if node is None:
                        continue
                    px, py, pz = node.getPosition()
                    if (math.hypot(px - task["x"], py - task["y"]) <= task["radius"]
                            and task.get("z_min", -1e9) <= pz <= task.get("z_max", 1e9)):
                        task["done"] = True
                        task["sim_time"] = round(t, 2)
                        done_n = sum(1 for k in self.man_tasks if k["done"])
                        print(f"[referee] task {task['name']} DONE (+{task['points']}) @ {t:.1f}s "
                              f"[{done_n}/{len(self.man_tasks)}]", flush=True)
                        if done_n == len(self.man_tasks) and self._door is not None:
                            self._door_z = 0.0          # start the door-open animation
                            print("[referee] all tasks done — door opening", flush=True)
                # sliding-door animation: 0.5 m/s downward until fully dropped
                if self._door_z is not None and self._door_z < self._door_drop:
                    step = 0.5 * self.ts / 1000.0
                    self._door_z = min(self._door_drop, self._door_z + step)
                    f = self._door.getField("translation")
                    v = f.getSFVec3f()
                    f.setSFVec3f([v[0], v[1], v[2] - step])
                    if self._door_z >= self._door_drop:
                        print("[referee] door OPEN", flush=True)

            # checkpoint scoring
            if self.order == "any":
                # collectibles: any unscored checkpoint in range scores immediately
                for cp in self.cps:
                    if not cp["scored"] and math.hypot(x - cp["x"], y - cp["y"]) <= cp["radius"]:
                        cp["scored"] = True
                        cp["sim_time"] = round(t, 2)
                        self._recolor(cp, GREEN)
                        done = sum(1 for c in self.cps if c["scored"])
                        print(f"[referee] {cp['name']} collected (+{cp['points']}) @ {t:.1f}s "
                              f"[{done}/{len(self.cps)}]", flush=True)
            elif self._idx < len(self.cps):
                cp = self.cps[self._idx]
                if math.hypot(x - cp["x"], y - cp["y"]) <= cp["radius"]:
                    cp["scored"] = True
                    cp["sim_time"] = round(t, 2)
                    self._recolor(cp, GREEN)
                    print(f"[referee] {cp['name']} scored (+{cp['points']}) @ {t:.1f}s", flush=True)
                    self._idx += 1
                    if self._idx < len(self.cps):
                        self._recolor(self.cps[self._idx], YELLOW)   # highlight next target

            # finish: hold inside the ring. Sequential mode arms it after the last checkpoint;
            # "any" mode arms it from the start (finish early with fewer points — your call).
            if self.order == "any" or all(c["scored"] for c in self.cps):
                fx, fy, fr = self.finish["x"], self.finish["y"], self.finish["radius"]
                if math.hypot(x - fx, y - fy) <= fr:
                    self._finish_hold += self.ts / 1000.0
                    if self._finish_hold >= self.finish.get("hold_s", 3.0):
                        self._recolor(self.finish, GREEN) if self.finish.get("def") else None
                        self._final("finish")
                        return
                else:
                    self._finish_hold = 0.0


if __name__ == "__main__":
    Referee().run()
