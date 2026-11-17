"""A 1U server in a small open-frame rack, and its front bezel: dimensions. Pure Python.

Rack frame: origin on the table under the middle of the server's front face; x into the rack (away from the arms), y to
the left, z up. The server sits at U4 of a 6U open-frame 19-inch rack. Its front bezel covers its front between the
two rack ears: the bezel's right end has two hook tabs that go into slots in the right ear; it then swings shut about
them until the spring latch at its left end clicks into the left ear.

Bezel frame: origin at its hook pivot (the right end, on its back face, half way up); x out of its back (into the
server), y along it to its left end, z up. Installed, it is the rack frame shifted to ``HOOK``.
"""

from __future__ import annotations

import math

# -- rack: 6U open-frame 19-inch rack, steel -------------------------------------------------------------------------
U = 0.04445
RACK_BASE = 0.015  # base frame height
RACK_U = 6
RACK_H = RACK_BASE + RACK_U * U + 0.015  # with the top frame
RACK_DEPTH = 0.50
POST = (0.016, 0.016)  # post section (x, y)
POST_Y = 0.2413 - 0.008  # the front posts' centres (19-inch rail spacing, 465 mm hole centres)
SERVER_U = 4  # the server's lowest U

# -- server: short-depth 1U, steel ------------------------------------------------------------------------------------
SERVER = (0.45, 0.434, 0.043)  # depth, width, height [m]
SERVER_Z = RACK_BASE + (SERVER_U - 1) * U  # its bottom
SERVER_ZC = SERVER_Z + SERVER[2] / 2
EAR = (0.003, 0.024, 0.043)  # rack ears on the front corners: x, y (from the chassis side out), z [m]
BAY = (0.10, 0.012)  # 2.5-inch drive bays across the front: width, height

# -- bezel: black plastic, 0.25 kg ------------------------------------------------------------------------------------
BEZEL = (0.025, 0.430, 0.042)  # depth, length, height [m]
BEZEL_MASS = 0.25
HOOK = (-0.0005, -SERVER[1] / 2 + 0.002, SERVER_ZC)  # the installed bezel's hook pivot, rack frame
TAB = (0.008, 0.006, 0.008)  # each hook tab, sticking out of the bezel's back at its right end

# -- what it does -------------------------------------------------------------------------------------------------------
HOOK_TOL = 0.003  # the hook tabs in their slots: the pivot within 3 mm of its place [m]
HOOK_TILT = math.radians(3.0)  # and the bezel within 3 degrees of upright, square to the server
HOOK_MIN = math.radians(5.0)  # and swung out at least this: the hook tabs go into their slots at an angle (pressed
# flat onto the front, they meet the ear beside their slots)
HOOK_OPEN = math.radians(40.0)  # swung out no more than this (the tabs bind past it)
LATCH = math.radians(0.6)  # swung shut to within this: the latch clicks (its left end within 4.5 mm)
HELD = {"lin": {"stiffness": 1.0e4, "damping": 60.0}, "ang": {"stiffness": 300.0, "damping": 3.0}}  # hooked, latched
CHAIN_MASS = 0.005  # the bezel's free motion is a chain of massless-ish joints: their bodies' mass [kg]
CHAIN_DAMPING = {"lin": 0.02, "ang": 0.0005}

# -- scoring ---------------------------------------------------------------------------------------------------------
REST_SPEED = 0.01  # m/s
