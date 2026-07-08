#!/usr/bin/env python3
"""policy_walk — the Go2's REAL locomotion in Webots: the firmware `free_walk` MNN policy
(extracted from the robot OS) drives the 12 leg joints with torque control under full physics.

This is a straight port of the verified walk core from `ai-robot/sim/controllers/go2_rl/go2_rl.py`
("0.5 m/s command walks ~0.45 m/s, upright, 9+ m stable"), with ONE structural change: the original
read base angular velocity / gravity via the Supervisor API; here they come from the robot's own
Gyro + InertialUnit devices, so the participant robot stays `supervisor FALSE` (it cannot cheat).

Control law (identical to the source):
  * physics tick (basicTimeStep, 4 ms): tau = clip(kp*(target − q) − kd*qd, ±tau_lim) → setTorque
  * policy tick (every 20 ms): obs = [cmd(3), ω_body(3), ĝ_body(3), q(12), qd(12), last_action(12)]
    normalized+clipped, stacked over `include_history_steps` frames (newest first) → MNN → action;
    target = act_mean + act_scale·action
  * first `settle` seconds: PD to init_jpos (lets the spawn transient die before the policy engages)

Postures (stand / crouch) are scripted PD keyframes, same style as the source's behaviours; the
policy resumes with a cleared history afterwards.

Requires numpy + MNN (see runtime.ini in the participant controller — Webots must run the venv
python). Deterministic: MNN session is forced to a single thread.
"""
import json
import math
import os
from collections import deque

import numpy as np
import MNN

HERE = os.path.dirname(os.path.abspath(__file__))
POLICY_DIR = os.environ.get("GO2_POLICY_DIR", os.path.join(HERE, "policies"))
LEGS = ["FL", "FR", "RL", "RR"]
JOINTS = [f"{l}_{j}_joint" for l in LEGS for j in ("hip", "thigh", "calf")]

# posture keyframes (firmware joint order FL,FR,RL,RR × hip,thigh,calf) — from the source
STAND = np.array([0.0, 0.79, -1.58] * 4, float)
CROUCH = np.array([0.0, 1.15, -2.10] * 4, float)


class _MNNPolicy:
    def __init__(self, path):
        self.it = MNN.Interpreter(path)
        # single thread → bit-deterministic inference (fair judging)
        self.sess = self.it.createSession({"numThread": 1})
        self.inp = self.it.getSessionInput(self.sess)
        self.n_in = int(np.prod(self.inp.getShape()))

    def __call__(self, obs):
        t = MNN.Tensor((1, self.n_in), MNN.Halide_Type_Float, obs.astype(np.float32).copy(),
                       MNN.Tensor_DimensionType_Caffe)
        self.inp.copyFrom(t)
        self.it.runSession(self.sess)
        out = self.it.getSessionOutput(self.sess)
        o = MNN.Tensor(out.getShape(), MNN.Halide_Type_Float, np.zeros(out.getShape(), np.float32),
                       MNN.Tensor_DimensionType_Caffe)
        out.copyToHostTensor(o)
        return np.array(o.getData(), np.float32).reshape(-1)[:12]


class PolicyWalk:
    """Firmware-policy locomotion for one Go2. Call `tick()` every simulation step
    (before robot.step()); set the velocity command with `command(vx, vy, vyaw)`.
    Note: command() resumes the walk policy immediately — a posture (stand/crouch) holds
    only while you DON'T call drive()/command() each loop iteration."""

    def __init__(self, robot, *, settle=0.8):
        with open(os.path.join(POLICY_DIR, "free_walk_export_cfg.json")) as f:
            cfg = json.load(f)
        self.H = cfg["include_history_steps"]
        self.clip_obs, self.clip_act = cfg["clip_obs"], cfg["clip_act"]
        self.kp = np.array(cfg["kp"]); self.kd = np.array(cfg["kd"])
        self.tau_lim = np.array(cfg["torque_limits"])
        self.init_q = np.array(cfg["init_jpos"])
        n = cfg["normalization"]
        self.act_mean = np.array(n["act_mean"]); self.act_scale = np.array(n["act_scale"])
        self.ob_mean = np.array(n["ob_mean"]); self.ob_scale = np.array(n["ob_scale"])
        self.policy = _MNNPolicy(os.path.join(POLICY_DIR, cfg["free_walk_model_name"]))

        self.robot = robot
        ts = int(robot.getBasicTimeStep())
        self.dt = ts / 1000.0
        self.decim = max(1, round(0.02 / self.dt))          # policy at 50 Hz
        self.settle = float(settle)

        self.motors, self.sensors = [], []
        for jn in JOINTS:
            m = robot.getDevice(jn)
            if m:
                m.setAvailableTorque(m.getMaxTorque())
            self.motors.append(m)
            s = robot.getDevice(jn + "_sensor")
            if s:
                s.enable(ts)
            self.sensors.append(s)
        self._gyro = robot.getDevice("gyro")
        self._imu = robot.getDevice("imu")
        if self._gyro:
            self._gyro.enable(ts)
        if self._imu:
            self._imu.enable(ts)
        if self.motors[0] and (self._gyro is None or self._imu is None):
            # a real robot without attitude feedback walks blind and WILL fall — shout.
            print("[policy_walk] WARNING: gyro/imu device missing — the locomotion policy "
                  "has no attitude feedback and will be unstable!", flush=True)

        self.cmd = np.zeros(3)                              # vx, vy, vyaw
        self.mode = "walk"                                  # walk | posture
        self._posture_goal = STAND
        self._posture_from = None
        self._posture_i = 0
        self._posture_steps = 1
        self.last_action = np.zeros(12)
        self.hist = deque(maxlen=self.H)
        self.q_prev = None                      # set on the first valid sensor sample
        self.target = self.act_mean.copy()
        self.t = 0.0
        self._step_i = 0

    # ── commands ────────────────────────────────────────────────────────────────
    def command(self, vx, vy, vyaw):
        if self.mode != "walk":                             # resume the policy cleanly
            self.mode = "walk"
            self.hist.clear()
            self.last_action = np.zeros(12)
        self.cmd = np.array([vx, vy, vyaw], float)

    def posture(self, goal, sec=0.6):
        """PD-lerp to a fixed pose (stand/crouch) and hold it; policy paused."""
        self.mode = "posture"
        self._posture_goal = np.asarray(goal, float)
        self._posture_from = None
        self._posture_i = 0
        self._posture_steps = max(1, int(sec / 0.02))

    def stand(self):
        if not (self.mode == "posture" and self._posture_goal is STAND):
            self.posture(STAND)

    def crouch(self, height=None):                          # height kept for API compat
        if not (self.mode == "posture" and self._posture_goal is CROUCH):
            self.posture(CROUCH, 0.8)

    # ── body-frame observation parts (device-based; no Supervisor) ───────────────
    def _base_parts(self):
        ang = np.array(self._gyro.getValues()) if self._gyro else np.zeros(3)
        if self._imu:
            r, p, y = self._imu.getRollPitchYaw()
            sr, cr = math.sin(r), math.cos(r)
            sp, cp = math.sin(p), math.cos(p)
            sy, cy = math.sin(y), math.cos(y)
            # body→world R = Rz(y)·Ry(p)·Rx(r); gravity in body frame = R^T·[0,0,-1]
            # = −(3rd row of R) = [sin p, −sin r·cos p, −cos r·cos p]
            grav = np.array([sp, -sr * cp, -cr * cp])
        else:
            grav = np.array([0.0, 0.0, -1.0])
        return ang, grav

    # ── the per-physics-tick update ──────────────────────────────────────────────
    def tick(self):
        self.t += self.dt
        q = np.array([s.getValue() if s else 0.0 for s in self.sensors])
        # Webots position sensors return NaN until the first robot.step() — skip those ticks
        # (motors keep zero torque; the settle-PD engages on the first valid sample).
        if not np.isfinite(q).all():
            self._step_i += 1
            return
        if self.q_prev is None:
            self.q_prev = q                     # first valid sample → qd = 0, no spike
        qd = (q - self.q_prev) / self.dt
        self.q_prev = q

        if self.t < self.settle:
            kp, kd, tau_lim = self.kp, self.kd, self.tau_lim
            self.target = self.init_q
        elif self.mode == "posture":
            if self._step_i % self.decim == 0:
                if self._posture_from is None:
                    self._posture_from = q.copy()
                f = min(1.0, (self._posture_i + 1) / self._posture_steps)
                self.target = self._posture_from + f * (self._posture_goal - self._posture_from)
                self._posture_i += 1
            kp, kd, tau_lim = np.full(12, 80.0), np.full(12, 1.0), self.tau_lim
        else:
            if self._step_i % self.decim == 0:
                ang, grav = self._base_parts()
                raw = np.concatenate([self.cmd, ang, grav, q, qd, self.last_action])
                frame = np.clip((raw - self.ob_mean) * self.ob_scale, -self.clip_obs, self.clip_obs)
                while len(self.hist) < self.H:
                    self.hist.append(frame.copy())
                self.hist.append(frame)
                action = np.clip(self.policy(np.concatenate(list(self.hist)[::-1])),
                                 -self.clip_act, self.clip_act)
                self.last_action = action
                self.target = self.act_mean + self.act_scale * action
            kp, kd, tau_lim = self.kp, self.kd, self.tau_lim

        tau = np.clip(kp * (self.target - q) - kd * qd, -tau_lim, tau_lim)
        for i, m in enumerate(self.motors):
            if m:
                m.setTorque(float(tau[i]))
        self._step_i += 1


if __name__ == "__main__":
    # offline self-test: cfg + model load, one inference on a neutral observation
    class _FakeRobot:
        def getBasicTimeStep(self):
            return 4
        def getDevice(self, name):
            return None
    pw = None
    try:
        pw = PolicyWalk(_FakeRobot())
    except Exception as e:  # noqa: BLE001
        print("policy_walk self-test FAILED:", e)
        raise SystemExit(1)
    frame = np.zeros(45)
    obs = np.concatenate([frame] * pw.H)
    a = pw.policy(obs)
    print(f"policy_walk self-test: H={pw.H} decim={pw.decim} action[0:4]={np.round(a[:4], 3)} "
          f"n_in={pw.policy.n_in} (expect {45 * pw.H}) — {'OK' if pw.policy.n_in == 45 * pw.H else 'MISMATCH'}")
