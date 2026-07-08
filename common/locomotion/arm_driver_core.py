#!/usr/bin/env python3
"""arm_driver_core — the FIXED controller of the stationary manipulator (ArmStation.proto).

Executes remote commands from the participant's controller (radio, ch 71) and streams telemetry
back (ch 72). Participants never edit this — they talk to it through the `robot.arm` client in
go2_api. The station is a Supervisor (it is NOT the participant robot), which lets it grab objects
KINEMATICALLY: while grabbed, the held object is pinned to the gripper tip each tick and its
velocity zeroed — deterministic and physics-stable (a magnetic Connector on a light ball driven by
stiff position-controlled joints blows up in ODE). On release the object is dropped with zero
velocity and rests where it was placed.

Graspable object DEF names are passed as controllerArgs (e.g. ["BALL_R" "BALL_G" "BALL_B"]).

Wire protocol (little-endian struct):
  in  (ch 71): b'M'+<3f x y z> move tip to a point in the ARM frame · b'G' grab · b'R' release
               b'S' stow · b'I' request RGB · b'D' request depth
  out (ch 72): b'T'+<3f tip><B holding><B at_target> every 8 ticks · b'I'/b'D' frames on request
"""
import struct
import sys

from controller import Supervisor

from arm_kinematics import ik, fk, JOINTS, STOW

TOL_REACHED = 0.05      # m — tip within this of the commanded point AND stopped → at_target
GRAB_RADIUS = 0.09      # m — a graspable this close to the tip latches on grab()


def _mat_vec(R, v):
    return (R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
            R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
            R[6]*v[0] + R[7]*v[1] + R[8]*v[2])


def run():
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    me = robot.getSelf()                                   # the station node (for its world pose)

    graspables = [robot.getFromDef(name) for name in sys.argv[1:]]
    graspables = [g for g in graspables if g is not None]

    motors, sensors = {}, {}
    for j in JOINTS:
        m = robot.getDevice(j)
        m.setVelocity(1.3)
        motors[j] = m
        s = robot.getDevice(j + "_sensor")
        s.enable(ts)
        sensors[j] = s
    cam = robot.getDevice("arm_camera")
    depth = robot.getDevice("arm_depth")
    cam_on = depth_on = False
    rx = robot.getDevice("cmd_rx")
    rx.enable(ts)
    tx = robot.getDevice("tel_tx")

    def apply(q):
        for j, v in zip(JOINTS, q):
            motors[j].setPosition(float(v))

    def q_now():
        return tuple(sensors[j].getValue() for j in JOINTS)

    def tip_world(tip_arm):
        # station world pose (identity rotation in the competition worlds, but be general)
        return tuple(a + b for a, b in zip(me.getPosition(), _mat_vec(me.getOrientation(), tip_arm)))

    target_tip = fk(STOW)
    apply(STOW)
    held = None                # the grabbed node, or None
    placed = []                # [(node, frozen_world_pos)] — released objects, pinned in place so
                               #   nothing (drift, a passing arm, a neighbour) can disturb them
    step_i = 0
    q_prev = None
    print(f"[arm_driver] station ready ({len(graspables)} graspables)", flush=True)

    while robot.step(ts) != -1:
        # ── commands ──
        while rx.getQueueLength() > 0:
            data = rx.getBytes()
            rx.nextPacket()
            if not data:
                continue
            op = data[:1]
            if op == b'M' and len(data) >= 13:
                x, y, z = struct.unpack("<3f", data[1:13])
                q = ik(x, y, z)
                target_tip = (x, y, z)
                if q is not None:
                    apply(q)
            elif op == b'G':
                if held is None:
                    tw = tip_world(fk(q_now()))
                    best, bestd = None, GRAB_RADIUS
                    for g in graspables:
                        gp = g.getPosition()
                        d = ((gp[0]-tw[0])**2 + (gp[1]-tw[1])**2 + (gp[2]-tw[2])**2) ** 0.5
                        if d < bestd:
                            best, bestd = g, d
                    held = best
                    placed[:] = [pl for pl in placed if pl[0] != held]   # re-grab a placed one
            elif op == b'R':
                if held is not None:
                    gp = held.getPosition()
                    placed.append((held, [gp[0], gp[1], gp[2]]))         # freeze where released
                    held.resetPhysics()
                    held = None
            elif op == b'S':
                apply(STOW)
                target_tip = fk(STOW)
            elif op == b'I':
                if not cam_on:
                    cam.enable(ts * 4); cam_on = True
                img = cam.getImage()
                if img:
                    tx.send(b'I' + struct.pack("<2H", cam.getWidth(), cam.getHeight()) + img)
            elif op == b'D':
                if not depth_on:
                    depth.enable(ts * 4); depth_on = True
                d = depth.getRangeImage(data_type="buffer")
                if d:
                    tx.send(b'D' + struct.pack("<2H", depth.getWidth(), depth.getHeight()) + bytes(d))

        # ── kinematic hold: pin the grabbed object just BELOW the tip, zero its velocity ──
        q = q_now()
        tip = fk(q)
        if held is not None:
            tw = tip_world(tip)
            held.getField("translation").setSFVec3f([tw[0], tw[1], tw[2] - 0.075])
            held.resetPhysics()
        # keep every released object frozen exactly where it was placed (drift/collision-proof)
        for node, pos in placed:
            node.getField("translation").setSFVec3f(pos)
            node.resetPhysics()

        # ── telemetry every 8 ticks ──
        if step_i % 8 == 0:
            holding = 1 if held is not None else 0
            moved = max((abs(a - b) for a, b in zip(q, q_prev)), default=1.0) if q_prev else 1.0
            q_prev = q
            near = sum((a - b) ** 2 for a, b in zip(tip, target_tip)) <= TOL_REACHED ** 2
            at_target = 1 if (near and moved < 0.02) else 0
            tx.send(b'T' + struct.pack("<3f2B", tip[0], tip[1], tip[2], holding, at_target))
        step_i += 1
