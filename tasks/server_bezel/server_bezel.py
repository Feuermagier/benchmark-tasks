"""A 1U server's front bezel lies on the bench: hook its right end into the rack ear and swing it shut until it latches.

The rack, the server and the bezel (``bezel/``: design, USD, hook and latch) are spawned through ``yampatch.py`` as one
fixed-base articulation: the bezel moves freely on a chain of joints until its hook tabs are in the right ear's slots,
then swings about them and latches. ``on_step`` runs the hook and latch and records the bezel's state for scoring;
``on_reset`` puts the bezel back on the bench. ``bezel/orient.py`` lets the scripted demo turn the grippers. The
docstring an agent reads is set at the end of this file.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/server_bezel/server_bezel.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --scorer task_score --max-steps 1800 \\
    --instruction "The drive swap is done. Put the server's front bezel back on."
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import yampatch  # noqa: E402
from bezel import design as d  # noqa: E402  (pure Python; usd and sim are imported after Isaac Sim starts)
from bezel import orient  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
RACK = np.array([0.42, 0.0, TOP])  # the rack frame's origin in the world: on the table under the server's front face
HOOK_W = RACK + np.array(d.HOOK)  # the installed bezel's hook pivot, world
LIE_X = 0.25  # the bezel lies face up on the table, its long axis along y, centred at this x and y 0
_DEBUG = bool(os.environ.get("BEZEL_DEBUG"))


def bezel_rotation(tilt: float, roll: float = 0.0, swing: float = 0.0) -> np.ndarray:
    cy, sy, cx, sx, cz, sz = (math.cos(tilt), math.sin(tilt), math.cos(roll), math.sin(roll), math.cos(swing),
                              math.sin(swing))  # fmt: skip
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return ry @ rx @ rz


# Lying face up (turned a quarter turn about y: its back down, its face up), its hook pivot just above the table.
START = {"free_x": LIE_X - float(HOOK_W[0]), "free_y": 0.0, "free_z": 0.0005 + TOP - float(HOOK_W[2]),
         "free_tilt": math.pi / 2, "free_roll": 0.0, "swing": 0.0}  # fmt: skip


def _make_rack():
    from bezel.sim import RackObject

    return RackObject("rack", tuple(float(v) for v in RACK), START)


OBJECTS = [yampatch.usd_object("rack", tuple(float(v) for v in RACK), kind="articulation", make=_make_rack,
                               extent=2 * d.POST_Y)]  # fmt: skip

# -- state -----------------------------------------------------------------------------------------------------------

_SIM = None  # bezel.sim.Rack, once the simulator runs
_STATE = {"hooked": False, "latched": False, "speed": 0.0, "step": 0}


def _sim(env):
    global _SIM
    if _SIM is None:
        from bezel.sim import Rack

        _SIM = Rack(env, "rack", START)
    return _SIM


def on_reset(env) -> None:
    _sim(env).reset()
    _STATE.update(hooked=False, latched=False, speed=0.0, step=0)


def on_step(env, states) -> None:
    sim = _sim(env)
    event = sim.after_step()
    _STATE.update(hooked=sim.hooked, latched=sim.latched, speed=sim.speed(), step=_STATE["step"] + 1)
    if _DEBUG and (event or _STATE["step"] % 10 == 0):
        q = sim.joints()
        lin = {k: round(q[k] * 1000, 1) for k in ("free_x", "free_y", "free_z")}
        ang = {k: round(math.degrees(q[k]), 2) for k in ("free_tilt", "free_roll", "swing")}
        print(f"[bezel] {_STATE['step']} {lin} {ang} hooked={sim.hooked} latched={sim.latched} {event or ''}",
              flush=True)  # fmt: skip


yampatch.install(on_reset=on_reset, on_step=on_step)

# -- scoring ---------------------------------------------------------------------------------------------------------


def score(objects):
    """0.4 for the bezel's right end hooked into the right ear; 1.0 for the bezel latched shut."""
    return 1.0 if _STATE["latched"] else 0.4 if _STATE["hooked"] else 0.0


def success(objects):
    return _STATE["latched"] and _STATE["speed"] < d.REST_SPEED


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Both arms pinch the bezel across its height near its ends, pointing down, lift it, and turn it upright together
# (its face towards the arms) while carrying it in front of the server, its left end swung out; they push its right
# end in so its hook tabs go into the right ear's slots, swing its left end in until the latch clicks, and let go.

orient.install()
GRIP_X = -(0.004 + PAD_TIP_PAST_TCP)  # the TCP in the bezel's frame: the pad tips 4 mm short of its back
GRIP_Y = {"right": 0.045, "left": d.BEZEL[1] - 0.045}
G_LOCAL = orient.grip((1, 0, 0), (0, 0, 1)).rot  # in the bezel's frame: pointing at its back, fingers across its height
SAG = 0.008  # the bezel sits this much low in the grippers' pinch when upright (the arms' give)
PINCH = -(d.BEZEL[2] / 2 - 0.003)  # finger target: pads 6 mm narrower than the bezel's height


def tcp(arm: str, pose: dict, back: float = 0.0):
    """The TCP position and grip for ``arm`` holding the bezel at ``pose`` (hook pivot offset, tilt, swing)."""
    r = bezel_rotation(pose["tilt"], 0.0, pose["swing"])
    lift = SAG * (1.0 - min(abs(pose["tilt"]) / (math.pi / 2), 1.0))  # held upright, aim above its sag in the grip
    p = HOOK_W + np.asarray(pose["p"], float) + r @ np.array([GRIP_X - back, GRIP_Y[arm], 0.0]) + np.array([0, 0, lift])
    return p, orient.Grip(r @ G_LOCAL)


def _wp(t, xyz, finger, label, grip):
    return Waypoint(float(t), tuple(float(v) for v in xyz), float(finger), label, grip)


def demo():
    lying = {"p": np.array([START["free_x"], 0.0, START["free_z"]]), "tilt": math.pi / 2, "swing": 0.0}
    lifted = {**lying, "p": lying["p"] + np.array([0.0, 0.0, 0.10])}
    out = math.radians(12.0)
    upright = {"p": np.array([-0.08, 0.0, 0.0]), "tilt": 0.0, "swing": 0.0}
    before = {"p": np.array([-0.04, 0.0, 0.0]), "tilt": 0.0, "swing": out}
    hooked = {**before, "p": np.zeros(3)}
    shut = {**hooked, "swing": -math.radians(1.0)}

    def path(a, b, n, t, label, finger):
        out_ = []
        for k in range(1, n + 1):
            s = k / n
            pose = {key: a[key] + (b[key] - a[key]) * s for key in ("p", "tilt", "swing")}
            out_.append((t / n, pose, finger, label))
        return out_

    steps = [(1.5, lying, FINGER_OPEN, "above the bezel's end", 0.06),
             (1.2, lying, FINGER_OPEN, "down beside it", 0.0),
             (0.6, lying, PINCH, "pinch it", 0.0)]
    seq = [(t, p, f, lab, up) for t, p, f, lab, up in steps]
    seq += [(t, p, f, lab, 0.0) for t, p, f, lab in path(lying, lifted, 3, 1.2, "lift it", PINCH)]
    seq += [(t, p, f, lab, 0.0) for t, p, f, lab in path(lifted, upright, 12, 3.6, "turn it upright, before the server",
                                                          PINCH)]  # fmt: skip
    seq += [(t, p, f, lab, 0.0) for t, p, f, lab in path(upright, before, 4, 1.2, "swing its left end out", PINCH)]
    seq += [(t, p, f, lab, 0.0) for t, p, f, lab in path(before, hooked, 5, 1.5, "right end into the ear", PINCH)]
    seq += [(t, p, f, lab, 0.0) for t, p, f, lab in path(hooked, shut, 8, 2.0, "swing it shut", PINCH)]
    seq += [(0.6, shut, FINGER_OPEN, "let go", 0.0), (0.4, shut, FINGER_OPEN, "hold", 0.0)]
    plans = {}
    for arm in ("left", "right"):
        plan = []
        for t, pose, finger, label, up in seq:
            p, g = tcp(arm, pose)
            plan.append(_wp(t, p + np.array([0.0, 0.0, up]), finger, label, g))
        p, g = tcp(arm, shut, back=0.06)
        plan.append(_wp(1.0, p, FINGER_OPEN, "back", g))
        plans[arm] = plan
    return plans


# -- what an agent reads ---------------------------------------------------------------------------------------------


def docs() -> str:
    return (
        f"A 6U open-frame 19-inch rack stands on the table, its front towards the arms at x {RACK[0]:.2f} m, centred at "
        f"y 0. A 1U server sits in it at U{d.SERVER_U}, its front (four drive bays, a rack ear at each side) "
        f"{d.SERVER_ZC:.2f} m above the table. Its black front bezel ({d.BEZEL[1] * 100:.0f} cm long) lies face up on "
        f"the table in front of the rack, centred at x {LIE_X:.2f} m, y 0, along y. The bezel's right end has two hook "
        "tabs that go into slots in the server's right rack ear; its left end has a spring latch that clicks into the "
        "left ear."
    )


__doc__ = docs()
