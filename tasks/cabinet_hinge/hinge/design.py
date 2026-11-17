"""A small wall cabinet whose door has come off its bottom clip-on hinge: dimensions. Pure Python.

Cabinet frame: origin at the middle of the carcass's front edge, at the underside of its bottom panel; x into the
cabinet (away from the arms), y to the left, z up. The door is hinged on the left side panel and stands open at 90
degrees, in front of that panel's front edge; its two concealed 35 mm cup hinges sit 80 mm from its top and bottom.
The top hinge's arm is clipped onto its mounting plate; the bottom hinge's arm has come off its plate and hangs on the
door, so the door hangs from one hinge and droops at its free edge.
"""

from __future__ import annotations

import math

# -- carcass: 16 mm melamine-faced chipboard, 4 mm hardboard back, on four 50 mm feet ---------------------------------
W, D, H = 0.30, 0.30, 0.40  # outside width (y), depth (x), height (z) [m]
T = 0.016  # panel thickness
BACK_T = 0.004
FEET_H = 0.05  # adjustable plastic feet
FOOT_R = 0.015
SIDE_IN_Y = W / 2 - T  # the left side panel's inner face

# -- door: 16 mm melamine-faced chipboard, full overlay, with a bar handle -------------------------------------------
DOOR = (0.296, 0.016, H)  # width, thickness, height [m]
DOOR_MASS = 1.30  # kg (about 690 kg/m3 chipboard)
HANDLE_PITCH = 0.128  # bar handle, 128 mm hole centres, standing 30 mm off the door, 12 mm bar
HANDLE_R = 0.006
HANDLE_STANDOFF = 0.030
HANDLE_FROM_EDGE = 0.035  # the handle's axis from the door's free edge
HANDLE_Z = 0.11  # the handle's centre above the door's bottom edge: low, as on a wall cabinet
HANDLE_MASS = 0.06

# -- hinges: 110-degree clip-on concealed cup hinges on cruciform mounting plates -------------------------------------
HINGE_FROM_END = 0.080  # cup centres from the door's top and bottom
Z_TOP, Z_BOT = H - HINGE_FROM_END, HINGE_FROM_END
CUP_R = 0.0175  # 35 mm cup
CUP_FROM_EDGE = 0.0215  # cup centre from the door's hinge edge
# The hinge axis with the door open at 90 degrees: 2 mm in front of the carcass and 2 mm outside its side panel
# (the door's inner face then lies 2 mm from the side panel, its hinge edge 4 mm in front of the carcass).
AXIS_X, AXIS_Y = -0.002, W / 2 + 0.002
PLATE = (0.050, 0.010, 0.014)  # the plate's centre bar: length along x, height off the side panel, width (z) [m]
PLATE_X = 0.037  # its front end from the carcass's front edge (the 37 mm hole line)
PLATE_WINGS = (0.016, 0.003, 0.042)  # its screw wings across the bar
ARM = (0.098, 0.009, 0.016)  # the hinge arm over the plate: length, thickness, width [m]
ARM_FRONT_X = 0.002  # the arm's front end, just inside the carcass (its crank goes round the panel's edge to the cup)
ARM_MASS = 0.08
KNUCKLE_MASS = 0.02

# -- what a single hinge lets the door do -----------------------------------------------------------------------------
# Hanging from its top hinge only, the door turns in its own plane about that hinge until the hinge's play and the
# cup's seat in the chipboard take its weight: its free edge droops 15 mm, and the bottom cup moves 13 mm into the
# cabinet, past its plate. The loose bottom arm turns on its own hinge pin, 12 degrees off its plate.
SAG_MAX = math.radians(3.0)  # the droop at rest
SAG_MIN = math.radians(-0.5)  # lifted past level, the top hinge's play the other way
SAG_DRIVE = {"stiffness": 10.0, "damping": 0.5}  # N m/rad: the top hinge's resistance before its play is taken up
ARM_OFF = math.radians(-12.0)  # the loose arm's rest angle on the door (negative: its rear away from the side panel)
ARM_DRIVE = {"stiffness": 0.3, "damping": 0.02}  # the hinge's own spring, N m/rad
SWING_DAMPING = 0.3  # N m s/rad: the hinges' friction
SWING_OPEN = math.radians(-20.0)  # the door opens to 110 degrees; 0 is 90 degrees open
SWING_SHUT = math.radians(89.5)  # shut (90 degrees is the door against the carcass)
# Clipped: the arm's hook is on the plate's front lug and its rear latch has snapped over the plate's tail.
CLIP_SAG = math.radians(0.35)  # the door level enough that the bottom arm's hook meets the lug: within 1.5 mm
CLIP_ANGLE = math.radians(0.8)  # the arm pressed flat onto the plate: its rear within 1.3 mm
# The latch: the arm's spring-loaded latch lever snaps over the plate's tail. Modelled as a sprung tongue on the plate's
# tail (between the arm's rear and the plate, as the lever is): it stands LATCH_PROUD off the plate's face, and the
# arm's rear must press it down to LATCH_CLICK (where the arm is within CLIP_ANGLE of flat) against CLICK_FORCE.
TONGUE = (0.012, 0.004, 0.010)  # x, y (thickness), z [m]
TONGUE_X = PLATE_X + PLATE[0] - 0.013  # its front end (x), on the plate's tail
TONGUE_MASS = 0.005
LATCH_PROUD = 0.0035  # m off the plate's face, at rest
LATCH_CLICK = 0.0011  # pressed to this, it clicks
CLICK_FORCE = 8.0  # N, at LATCH_CLICK: a firm finger press (makers publish no figure)
TONGUE_DRIVE = {"stiffness": CLICK_FORCE / (LATCH_PROUD - LATCH_CLICK), "damping": 20.0}  # about 3,300 N/m
CLIP_STILL = math.radians(5.0)  # and the door held still while it is pressed: swinging under 5 degrees/s
CLIPPED_CLEAR = math.radians(1.5)  # a clipped arm's rear 2.5 mm off the plate's face (the latch's own play)
CLIPPED_DRIVE = {"stiffness": 2000.0, "damping": 20.0}  # N m/rad: a clipped hinge holds the door

# -- scoring ---------------------------------------------------------------------------------------------------------
SHUT_TOL = math.radians(3.0)  # the door shut: within 3 degrees of closed
REST_SPEED = math.radians(2.0)  # rad/s
