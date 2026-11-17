"""A cabinet door has come off its bottom clip-on hinge: lift it level, clip the hinge back on, shut the door.

The cabinet (``hinge/``: design, USD, the clip) is spawned through ``yampatch.py`` as a fixed-base articulation: the
carcass with both mounting plates and the clipped top hinge, the door on the top hinge (it swings, and droops in its
own plane), and the loose bottom arm on the door. ``on_step`` runs the bottom hinge's clip and records the joints for
scoring; ``on_reset`` puts the door back. ``hinge/orient.py`` lets the scripted demo point the grippers forward. The
docstring an agent reads is set at the end of this file.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/cabinet_hinge/cabinet_hinge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --scorer task_score --max-steps 1800 \\
    --instruction "The cabinet door has come off its bottom hinge. Put it back on and shut the door."
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import yampatch  # noqa: E402
from hinge import design as d  # noqa: E402  (pure Python; usd and sim are imported after Isaac Sim starts)
from hinge import orient  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CAB = np.array([0.51, 0.0, TOP + d.FEET_H])  # the cabinet frame's origin in the world: its front edge, bottom panel
_DEBUG = bool(os.environ.get("HINGE_DEBUG"))


def _make_cabinet():
    from hinge.sim import CabinetObject

    return CabinetObject("cabinet", tuple(float(v) for v in CAB))


OBJECTS = [
    yampatch.usd_object("cabinet", tuple(float(v) for v in CAB), kind="articulation", make=_make_cabinet,
                        extent=d.W),
]  # fmt: skip

# -- state -----------------------------------------------------------------------------------------------------------

_SIM = None  # hinge.sim.Cabinet, once the simulator runs
_STATE = {"q": None, "clipped": False, "speed": 0.0, "step": 0}


def _sim(env):
    global _SIM
    if _SIM is None:
        from hinge.sim import Cabinet

        _SIM = Cabinet(env, "cabinet")
    return _SIM


def on_reset(env) -> None:
    _sim(env).reset()
    _STATE.update(q=None, clipped=False, speed=0.0, step=0)


def on_step(env, states) -> None:
    sim = _sim(env)
    event = sim.after_step()
    _STATE.update(q=sim.joints(), clipped=sim.clipped, speed=sim.swing_speed(), step=_STATE["step"] + 1)
    if _DEBUG and (event or _STATE["step"] % 10 == 0):
        q = {k: round(v * 1000, 2) if k == "tongue" else round(math.degrees(v), 2) for k, v in _STATE["q"].items()}
        print(f"[hinge] {_STATE['step']} {q} clipped={sim.clipped} {event or ''}", flush=True)


yampatch.install(on_reset=on_reset, on_step=on_step)

# -- scoring ---------------------------------------------------------------------------------------------------------


def clipped() -> bool:
    return bool(_STATE["clipped"])


def shut() -> bool:
    q = _STATE["q"]
    return q is not None and q["swing"] > d.SWING_SHUT - d.SHUT_TOL


def score(objects):
    """0.5 for the bottom hinge clipped back on its plate; 1.0 for that and the door shut."""
    return 0.5 * clipped() + 0.5 * (clipped() and shut())


def success(objects):
    return score(objects) == 1.0 and _STATE["speed"] < d.REST_SPEED


# -- scripted demo ---------------------------------------------------------------------------------------------------
# The left arm, pointing forward, pinches the door's free edge half way up and lifts it along the arc it droops
# on, until the door hangs level; the right arm, pointing into the cabinet with its fingers closed, reaches past the
# loose bottom arm and presses its rear onto the plate (it snaps on). The left arm then lets go; the right arm goes round
# the door's free edge and pushes it shut by its outer face.

orient.install()
AXIS_TOP = CAB + np.array([d.AXIS_X, d.AXIS_Y, d.Z_TOP])
AXIS_BOT = CAB + np.array([d.AXIS_X, d.AXIS_Y, d.Z_BOT])
EDGE = d.AXIS_X - 0.002 - d.DOOR[0]  # the free edge, door frame: x from the hinge axis
FORWARD = orient.grip((1, 0, 0), (0, -1, 0))  # pointing into the cabinet, fingers closing across y
DOOR_PINCH = -(d.DOOR[1] / 2 - 0.006)  # finger target: pads 12 mm narrower than the door, a firm pinch
FINGER_HALF = 0.0092  # closed fingertips: their outer faces this far either side of the TCP (the fingers' roots
# are about 4 mm wider)
GRIP_IN = 0.025  # the pad tips this far past the free edge
PINCH_DZ = d.DOOR[2] / 2 - d.Z_TOP  # the pinch's height on the door, from the top hinge: half way up
LIFT_TO = -math.radians(3.0)  # the lift's aim: past level (the door stops a little past level, at its play)
PARK = (0.15, -0.20, TOP + 0.15)  # the right arm's wait, outside the door's swing and in front of its outer face
PRESS = orient.grip((1, 0, 0), (0, 0, 1))  # pressing: pointing into the cabinet, fingers closed vertically (it pushes with
# a finger's side, square to the fingers' travel)
PRESS_HALF = 0.0137  # the closed fingers' sides this far either side of the TCP, across their travel
PRESS_IN = 0.05  # the press's aim past the plate's face: the arm's position gains are soft, so a firm push aims well past


def _wp(t, xyz, finger, label, yaw=0.0):
    return Waypoint(float(t), tuple(float(v) for v in xyz), float(finger), label, yaw)


def _duration(plan) -> float:
    return sum(w.duration for w in plan)


def _wait_until(plan, t, label="wait") -> None:
    if _duration(plan) < t:
        last = plan[-1]
        plan.append(Waypoint(t - _duration(plan), last.pos, last.finger, label, last.yaw))


def door_point(dx: float, dy: float, dz: float, sag: float = 0.0, swing: float = 0.0) -> np.ndarray:
    """A point of the door (from the top hinge's axis: x along the door, y off its inner face, z up), in the world."""
    c, s = math.cos(sag), math.sin(sag)
    x, z = dx * c - dz * s, dx * s + dz * c  # drooped in the door's plane about the top hinge
    cw, sw = math.cos(swing), math.sin(swing)
    return AXIS_TOP + np.array([x * cw - dy * sw, x * sw + dy * cw, z])


def _left() -> tuple[list[Waypoint], float, float]:
    """The door: pinch, lift level, (wait), swing towards shut, let go. Returns the plan, the time the door is level
    and the index to wait at."""
    tcp_dx = EDGE + GRIP_IN - PAD_TIP_PAST_TCP
    mid = d.DOOR[1] / 2
    at = lambda sag, swing=0.0, back=0.0: door_point(tcp_dx - back, mid, PINCH_DZ, sag, swing)  # noqa: E731
    aside = at(d.SAG_MAX, back=0.045) + np.array([0.0, 0.09, 0.0])
    plan = [_wp(1.6, aside, FINGER_OPEN, "beside the door's free edge", FORWARD),
            _wp(1.0, at(d.SAG_MAX, back=0.045), FINGER_OPEN, "before it", FORWARD),
            _wp(1.2, at(d.SAG_MAX), FINGER_OPEN, "round the edge", FORWARD),
            _wp(0.6, at(d.SAG_MAX), DOOR_PINCH, "pinch it", FORWARD)]  # fmt: skip
    for k in range(1, 9):  # lift along the droop's arc, aiming a little past level against the arm's own give
        plan.append(_wp(0.3, at(d.SAG_MAX + (LIFT_TO - d.SAG_MAX) * k / 8), DOOR_PINCH, "lift level", FORWARD))
    level = _duration(plan)
    return plan, level, at


def demo():
    left, level, at = _left()
    sag_hold = LIFT_TO
    # The right arm: in past the loose arm's rear, then sideways onto it.
    al, at_, aw = d.ARM
    face_y = d.SIDE_IN_Y - d.PLATE[1] - at_  # a seated arm's inner face, cabinet frame
    press_x = CAB[0] + 0.078 - PAD_TIP_PAST_TCP + 0.010  # where the finger's side meets the arm's rear half
    z = AXIS_BOT[2]
    clear_y = CAB[1] + 0.072
    right = [_wp(2.0, (CAB[0] - 0.10, clear_y, z), FINGER_CLOSED, "before the cabinet", PRESS)]
    _wait_until(right, level + 0.3, "wait for the door")
    right += [_wp(1.5, (press_x, clear_y, z), FINGER_CLOSED, "in, past the loose arm", PRESS),
              _wp(1.2, (press_x, CAB[1] + face_y - PRESS_HALF + PRESS_IN, z), FINGER_CLOSED, "press it onto the plate",
                  PRESS),
              _wp(0.6, (press_x, CAB[1] + face_y - PRESS_HALF + PRESS_IN, z), FINGER_CLOSED, "hold", PRESS),
              _wp(0.5, (press_x, clear_y, z), FINGER_CLOSED, "off it", PRESS),
              _wp(1.5, (CAB[0] - 0.12, clear_y, z), FINGER_CLOSED, "out", PRESS),
              _wp(1.5, PARK, FINGER_CLOSED, "clear of the door's swing", FORWARD)]  # fmt: skip
    clipped_at = _duration(right) - 3.5
    _wait_until(left, clipped_at, "hold the door level")
    # Shut the door: the left arm lets go and moves away; the right arm, tilted forward with its fingers closed, goes
    # round the door's free edge and sweeps it shut by its outer face, low down.
    out = d.DOOR[1] + FINGER_HALF - 0.002  # the closed fingers' side 2 mm into the face (they are a little wider)
    push = orient.grip((0.87, 0.0, -0.5), (0.0, -1.0, 0.0))
    rr, rz = 0.18, TOP + d.FEET_H + 0.06

    def face(sw, off=0.0, z=rz):
        p = door_point(-rr, out + off, 0.0, 0.0, sw)
        return (p[0], p[1], z)

    off_edge = at(sag_hold, back=0.05)
    left += [_wp(0.5, at(sag_hold), FINGER_OPEN, "let go", FORWARD),
             _wp(0.8, off_edge, FINGER_OPEN, "off the edge", FORWARD),
             _wp(1.5, (0.24, 0.27, TOP + 0.30), FINGER_OPEN, "away", FORWARD)]  # fmt: skip
    released = _duration(left) - 1.5
    _wait_until(right, released, "wait for the door")
    round_x = CAB[0] + EDGE - 0.06  # in front of the free edge
    right += [_wp(1.2, (round_x, -0.05, rz), FINGER_CLOSED, "in front of the door", push),
              _wp(1.5, (round_x, face(0.0, 0.05)[1], rz), FINGER_CLOSED, "round its free edge", push),
              _wp(1.2, face(0.0, 0.05), FINGER_CLOSED, "beside its outer face", push),
              _wp(0.6, face(0.0), FINGER_CLOSED, "to it", push)]  # fmt: skip
    end, n = math.radians(92.0), 20
    for k in range(1, n + 1):
        right.append(_wp(0.3, face(end * k / n), FINGER_CLOSED, "push it shut", push))
    right += [_wp(0.5, face(end), FINGER_CLOSED, "hold", push),
              _wp(1.0, face(end, 0.05, rz + 0.04), FINGER_CLOSED, "away", push)]  # fmt: skip
    _wait_until(left, _duration(right))
    return {"left": left, "right": right}


# -- what an agent reads ---------------------------------------------------------------------------------------------


def docs() -> str:
    return (
        f"A small white wall cabinet ({d.W * 100:.0f} cm wide, {d.D * 100:.0f} cm deep, {d.H * 100:.0f} cm tall, on "
        f"{d.FEET_H * 100:.0f} mm feet) stands on the table with its open front towards the arms, its front edge at "
        f"x {CAB[0]:.2f} m, centred at y {CAB[1]:+.2f} m. Its door is hinged on the left side panel and stands open "
        "at 90 degrees, in front of that panel. The door hangs from its top hinge only: the bottom hinge's arm has come "
        "off its mounting plate inside the cabinet, so the door droops at its free edge and the loose arm sits off "
        "its plate. The door has a bar handle near its free edge."
    )


__doc__ = docs()
