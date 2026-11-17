"""Change a stopped wall clock's AA cell: the flat one out into the used-battery tray, the new one in, + to +.

The clock (``clock/``: design, USD, spring) is spawned through ``yampatch.py`` as an articulation: its case, the
quartz movement and battery holder on its back, and the holder's spring on a prismatic drive. The cells are rigid USD
bodies, the tray a fixed one. ``on_step`` records the clock's pose for scoring, ``on_reset`` puts the clock and the
flat cell back. The docstring an agent reads is set at the end of this file.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/clock_battery/clock_battery.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --scorer task_score --max-steps 1200 \\
    --instruction "The wall clock has stopped. Put the new battery in."
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

# Task files are imported by path, so this folder is not on sys.path: add it for yampatch.py and the clock package.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import yampatch  # noqa: E402
from clock import design as d  # noqa: E402  (pure Python; usd and sim are imported after Isaac Sim starts)

SCENE = "station"
TOP = TABLE_TOP[SCENE]
REST_GAP = 0.0005
CLOCK_XY = (0.32, -0.06)  # the clock's centre; it lies face down, its holder's + end towards +x
NEW_XY = (0.17, -0.25)  # the new cell in its opened pack on the table, + end towards +y
NEW_Z = d.PACK_CARD[2] + d.CELL_Z  # its centre above the table
TRAY_XY = (0.43, -0.27)
SEAT = np.array([d.PLUS_FACE_X - d.AA_LEN / 2, 0.0, d.FLOOR_Z + d.CELL_Z])  # a seated cell's centre, clock frame
CAN_OFFSET = d.AA_NUB[1] / 2  # a cell USD's origin (the can's centre) lies this far behind the cell's centre [m]
_DEBUG = bool(os.environ.get("CLOCK_DEBUG"))
YAW_Y = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # a quarter turn: + end towards +y


def _make_clock():
    from clock.sim import ClockObject

    return ClockObject("clock", (*CLOCK_XY, TOP + REST_GAP))


def _build(what: str, *args):
    def build():
        from clock import usd

        return getattr(usd, f"build_{what}")(*args)

    return build


_OLD_POS = (float(CLOCK_XY[0] + SEAT[0] - CAN_OFFSET), CLOCK_XY[1], float(TOP + REST_GAP + SEAT[2]))
OBJECTS = [
    yampatch.usd_object("clock", (*CLOCK_XY, TOP + REST_GAP), kind="articulation", make=_make_clock,
                        extent=2 * d.CLOCK_R),
    yampatch.usd_object("old_cell", _OLD_POS, kind="rigid", build=_build("battery", False), mass=d.AA_MASS,
                        extent=d.AA_LEN),
    yampatch.usd_object("pack", (*NEW_XY, TOP), kind="fixed", build=_build("pack"), extent=d.PACK_CARD[1]),
    yampatch.usd_object("new_cell", (NEW_XY[0], NEW_XY[1] - CAN_OFFSET, TOP + NEW_Z + REST_GAP), kind="rigid",
                        build=_build("battery", True), rot_xyzw=YAW_Y, mass=d.AA_MASS, extent=d.AA_LEN),
    yampatch.usd_object("tray", (*TRAY_XY, TOP), kind="fixed", build=_build("tray"), extent=d.TRAY[0]),
]  # fmt: skip

# -- state -----------------------------------------------------------------------------------------------------------

_SIM = None  # clock.sim.Clock, once the simulator runs
_STATE = {"clock": None}  # the clock's (position, quaternion) after the last step


def _sim(env):
    global _SIM
    if _SIM is None:
        from clock.sim import Clock

        _SIM = Clock(env, "clock", "old_cell")
    return _SIM


def on_reset(env) -> None:
    _sim(env).reset()
    _STATE.update(clock=None, step=0)


def on_step(env, states) -> None:
    sim = _sim(env)
    _STATE["clock"] = sim.pose()
    if _DEBUG:
        _STATE["step"] = _STATE.get("step", 0) + 1
        if _STATE["step"] % 10 == 0:
            cells = " ".join(f"{n}={np.round(states[n].pos, 4).tolist()}" for n in ("old_cell", "new_cell"))
            print(f"[clock] {_STATE['step']} spring={sim.spring_pos():.4f} {cells} seated={seated(states['new_cell'])}",
                  flush=True)  # fmt: skip


yampatch.install(on_reset=on_reset, on_step=on_step)

# -- scoring ---------------------------------------------------------------------------------------------------------


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def seated(cell) -> int:
    """+1 if ``cell`` sits in the holder + to +, -1 if it sits there the wrong way round, else 0. Judged from its
    bounding-box centre (the cell's centre) and axis in the clock's frame."""
    if _STATE["clock"] is None:
        return 0
    c, q = _STATE["clock"]
    r = _rot(q)
    rel = r.T @ (np.asarray(cell.pos) - c) - SEAT
    if abs(rel[0]) > d.SEAT_TOL or abs(rel[1]) > d.SEAT_TOL or abs(rel[2]) > d.SEAT_Z_TOL:
        return 0
    align = float((r.T @ _rot(cell.quat_xyzw)[:, 0])[0])  # the cell's + direction along the holder's +x
    return 1 if align > d.SEAT_ALIGN else -1 if align < -d.SEAT_ALIGN else 0


def in_tray(cell) -> bool:
    """Inside the tray's walls, below its rim."""
    inner = np.array(d.TRAY[:2]) / 2 - d.TRAY_WALL
    p = np.asarray(cell.pos)
    return bool(np.all(np.abs(p[:2] - np.array(TRAY_XY)) < inner) and p[2] < TOP + d.TRAY[2])


def score(objects):
    """0.4 for the flat cell in the used-battery tray, 0.6 for the new cell seated in the holder + to +."""
    return round(0.4 * in_tray(objects["old_cell"]) + 0.6 * (seated(objects["new_cell"]) == 1), 6)


def success(objects):
    """Both done, both cells at rest."""
    return score(objects) == 1.0 and objects["old_cell"].resting() and objects["new_cell"].resting()


# -- scripted demo ---------------------------------------------------------------------------------------------------
# The left arm presses the clock's back down near its rim, so it neither slides nor rocks. The right arm pinches the
# flat cell across its middle, lifts it out (the spring lets go of it) and drops it in the tray; takes the new cell,
# turning it a quarter turn so its + end points at the holder's + contact, and lowers it with its - end over the
# spring's chamfer: pressing it down pushes the spring back and the cell snaps in between the contacts.

CX, CY = CLOCK_XY
Z0 = TOP + REST_GAP
HOVER = TOP + 0.105
SEAT_W = np.array([CX + SEAT[0], CY, Z0 + SEAT[2]])  # a seated cell's centre, world
PAD_LOW = d.CELL_Z - 0.0015  # pad tips 1.5 mm above what the cell lies on
CELL_PINCH = -(d.AA_R - 0.002)  # finger target: pads 4 mm narrower than the cell, a firm pinch
MINUS_PRESS = 0.016  # where the fingertips press the new cell's - end, from the holder's centre [m]
HOLD_XY = (CX + 0.06, CY + 0.135)  # near the rim, towards the left arm
PRESS_Z = Z0 + d.CLOCK_DEPTH - 0.002 + PAD_TIP_PAST_TCP  # fingertips 2 mm into the clock's back: a firm press


def _wp(t, xyz, finger, label, yaw=0.0):
    return Waypoint(t, tuple(float(v) for v in xyz), finger, label, float(yaw))


def _duration(plan) -> float:
    return sum(w.duration for w in plan)


def _wait_until(plan, t, label="wait") -> None:
    if _duration(plan) < t:
        last = plan[-1]
        plan.append(Waypoint(t - _duration(plan), last.pos, last.finger, label, last.yaw))


def _at_cell(x, y, centre_z, finger, label, yaw=0.0):
    """The TCP for a pinch on a cell whose centre is at ``centre_z``."""
    return _wp(0.0, (x, y, centre_z - PAD_LOW + PAD_TIP_PAST_TCP), finger, label, yaw)


def _t(w: Waypoint, t: float) -> Waypoint:
    return Waypoint(t, w.pos, w.finger, w.label, w.yaw)


def demo():
    hold = [_wp(1.5, (*HOLD_XY, HOVER), FINGER_CLOSED, "above the clock's back"),
            _wp(1.2, (*HOLD_XY, PRESS_Z), FINGER_CLOSED, "press the clock down")]
    sx, sy, sz = SEAT_W
    nx, ny = NEW_XY
    table_z = TOP + NEW_Z
    work = [_wp(1.5, (sx, sy, HOVER), FINGER_OPEN, "above the flat cell")]
    _wait_until(work, _duration(hold), "wait for the hold")
    work += [_t(_at_cell(sx, sy, sz, FINGER_OPEN, "down beside it"), 1.5),
             _t(_at_cell(sx, sy, sz, CELL_PINCH, "pinch it"), 0.6),
             _wp(1.2, (sx, sy, HOVER), CELL_PINCH, "lift it out"),
             _wp(1.5, (*TRAY_XY, TOP + 0.09), CELL_PINCH, "over the tray"),
             _wp(0.5, (*TRAY_XY, TOP + 0.09), FINGER_OPEN, "drop it in"),
             _wp(1.5, (nx, ny, HOVER), FINGER_OPEN, "above the new cell", math.pi / 2),
             _t(_at_cell(nx, ny, table_z, FINGER_OPEN, "down beside it", math.pi / 2), 1.2),
             _t(_at_cell(nx, ny, table_z, CELL_PINCH, "pinch it", math.pi / 2), 0.6),
             _wp(1.0, (nx, ny, HOVER), CELL_PINCH, "lift it", math.pi / 2),
             _wp(2.0, (sx, sy, HOVER), CELL_PINCH, "over the holder, + end to +", 0.0),
             _t(_at_cell(sx + 0.001, sy, sz + 0.006, CELL_PINCH, "lower it: - end onto the spring"), 1.5),
             _t(_at_cell(sx + 0.001, sy, sz + 0.006, FINGER_OPEN, "let go"), 0.6),
             _wp(0.8, (sx, sy, HOVER), FINGER_OPEN, "up")]
    # The fingertips, 10 mm apart, straddle the top of the - end (which keeps it centred) and press it down past the
    # spring's chamfer.
    px, straddle = sx - MINUS_PRESS, -0.005
    top = sz + math.sqrt(d.AA_R**2 - 0.005**2)  # where the fingertips meet a seated cell's flanks
    work += [_wp(1.0, (px, sy, top + 0.015 + PAD_TIP_PAST_TCP), straddle, "over the - end"),
             _wp(1.2, (px, sy, top - 0.002 + PAD_TIP_PAST_TCP), straddle, "press the - end down"),
             _wp(0.5, (px, sy, top - 0.002 + PAD_TIP_PAST_TCP), straddle, "hold"),
             _wp(1.0, (px, sy, HOVER), straddle, "up")]  # fmt: skip
    _wait_until(hold, _duration(work), "hold the clock")
    hold.append(_wp(1.0, (*HOLD_XY, HOVER), FINGER_CLOSED, "up"))
    _wait_until(work, _duration(hold))
    return {"left": hold, "right": work}


# -- what an agent reads ---------------------------------------------------------------------------------------------


def docs() -> str:
    return (
        f"A {2 * d.CLOCK_R * 100:.0f} cm round quartz wall clock lies face down on the table, centred at "
        f"({CX:.2f}, {CY:+.2f}) m. On its back, the battery holder of its movement holds a flat AA cell between the "
        f"+ contact and the - spring, the holder marked + and - at its ends. A new AA cell (black, copper + end) lies "
        f"in its opened blister pack on the table at ({NEW_XY[0]:.2f}, {NEW_XY[1]:+.2f}) m; a green tray labelled "
        f"USED BATTERIES stands at "
        f"({TRAY_XY[0]:.2f}, {TRAY_XY[1]:+.2f}) m."
    )


__doc__ = docs()
