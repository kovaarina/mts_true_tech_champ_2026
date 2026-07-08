#!/usr/bin/env python3
"""Pure kinematics of the stationary competition arm (ArmStation.proto). No Webots imports.

Arm frame: origin at the pedestal centre on the FLOOR, z up. Shoulder S=(0,0,0.36);
links L1=0.30, L2=0.28, wrist->tip W=0.12. Pitch is positive-DOWN (Webots +y hinges).
IK keeps the gripper vertical (top grasp): q_wrist = pi/2 - q_sh - q_el.
"""
import math

S = (0.0, 0.0, 0.61)
L1, L2, W = 0.30, 0.28, 0.12
JOINTS = ("arm_base", "arm_shoulder", "arm_elbow", "arm_wrist")
LIMITS = {"arm_base": (-3.1, 3.1), "arm_shoulder": (-1.6, 1.6),
          "arm_elbow": (-2.8, 0.1), "arm_wrist": (-2.9, 2.9)}
STOW = (0.0, -1.20, -2.40, 1.10)


def ik(x, y, z):
    """Joint targets for tip at (x,y,z), gripper pointing down; None if unreachable."""
    dx, dy, dz = x - S[0], y - S[1], z - S[2]
    base = math.atan2(dy, dx)
    r = math.hypot(dx, dy)
    wr, wh = r, dz + W
    D = math.hypot(wr, wh)
    if D < 1e-6 or D > L1 + L2 - 1e-4 or D < abs(L1 - L2) + 1e-4:
        return None
    ca = max(-1.0, min(1.0, (D*D - L1*L1 - L2*L2) / (2*L1*L2)))
    alpha = math.acos(ca)
    for el in (-alpha, alpha):
        sh = math.atan2(-wh, wr) - math.atan2(L2*math.sin(el), L1 + L2*math.cos(el))
        wrist = math.pi/2 - sh - el
        q = (base, sh, el, wrist)
        if all(LIMITS[j][0] <= v <= LIMITS[j][1] for j, v in zip(JOINTS, q)):
            return q
    return None


def fk(q):
    base, sh, el, wr = q
    p1, p2, p3 = sh, sh + el, sh + el + wr
    r = L1*math.cos(p1) + L2*math.cos(p2) + W*math.cos(p3)
    h = -L1*math.sin(p1) - L2*math.sin(p2) - W*math.sin(p3)
    return (S[0] + r*math.cos(base), S[1] + r*math.sin(base), S[2] + h)


if __name__ == "__main__":
    import random
    random.seed(1)
    bad = tested = 0
    for _ in range(20000):
        t = (random.uniform(-0.6, 0.6), random.uniform(-0.6, 0.6), random.uniform(0.05, 0.8))
        q = ik(*t)
        if q is None:
            continue
        tested += 1
        if math.dist(fk(q), t) > 1e-9:
            bad += 1
    q = ik(0.40, 0.10, 0.20)
    pitch_ok = q and abs((q[1]+q[2]+q[3]) - math.pi/2) < 1e-9
    print(f"arm_kinematics: {tested} reachable, errors={bad}, top-grasp={pitch_ok} -> "
          f"{'PASS' if bad == 0 and pitch_ok and tested > 3000 else 'FAIL'}")
