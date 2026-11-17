"""A quartz wall clock lying face down, its AA cell, the spare and the used-battery tray: dimensions. Pure Python.

Clock frame: origin at the centre of the clock's rim where it rests on the table (the clock lies face down), z up
(out of its back), x and y in the plane of the face. The quartz movement is a box on the back, centre; its battery
holder is on top of the movement: a cradle for one AA cell along x, the + contact at +x, the spring (-) at -x.
"""

from __future__ import annotations

# -- clock -----------------------------------------------------------------------------------------------------------
CLOCK_R = 0.15  # 30 cm round wall clock
CLOCK_DEPTH = 0.045  # rim to back [m]
CLOCK_SIDES = 32  # its round case as a polygon
CLOCK_MASS = 0.45  # kg: plastic case, glass, movement
MOVEMENT = (0.056, 0.056, 0.016)  # quartz movement box on the back [m]
FLOOR_Z = CLOCK_DEPTH + MOVEMENT[2]  # the holder's floor (the movement's top), clock frame

# -- AA cell (IEC LR6) -----------------------------------------------------------------------------------------------
AA_R = 0.00725  # 14.5 mm diameter
AA_BODY = 0.049  # can length [m]
AA_NUB = (0.0027, 0.0015)  # + terminal: radius, height [m]
AA_LEN = AA_BODY + AA_NUB[1]  # 50.5 mm
AA_MASS = 0.023  # kg

# -- holder ----------------------------------------------------------------------------------------------------------
# The cell rests on two saddles near its ends, over a finger cut-out in the movement: fingertips reach below its
# middle from either side.
SADDLE = (0.004, 0.014, 0.005)  # each saddle (x, y, z) [m]
SADDLE_LIP = (0.004, 0.002, 0.0015)  # a lip along each side of a saddle's top, keeping the cell from rolling [m]
SADDLE_X = 0.0225  # saddle centres, either side of the holder's centre: 41 mm clear between them [m]
CELL_Z = SADDLE[2] + AA_R  # a resting cell's centre above the holder's floor
PLUS_FACE_X = AA_LEN / 2  # the + contact's face: a seated cell is centred on the holder
SPRING_REST_X = -AA_LEN / 2 + 0.0035  # the spring's face at rest: a seated cell compresses it 3.5 mm
SPRING_T = 0.008  # the spring, as a plate with a 45-degree chamfer on top that a cell pressed down onto pushes back [m]
SPRING_FACE_H = 0.012  # the face's upright part, from the floor
SPRING_H = SPRING_FACE_H + SPRING_T
SPRING_TRAVEL = 0.010  # m
SPRING_DRIVE = {"stiffness": 150.0, "damping": 0.3, "armature": 1e-5}  # N/m, N s/m, kg: a light coil spring
SPRING_PRELOAD = 0.001  # m past its rest stop: 0.15 N with no cell in
SPRING_MASS = 0.002
END_WALL = (0.003, 0.018, 0.022)  # the + contact block and the spring's backing block (x, y, z) [m]

# -- the new cell's opened retail blister pack: a card with the same two saddles -------------------------------------
PACK_CARD = (0.065, 0.10, 0.0015)  # x, y, z [m]; the cell lies along y

# -- used-battery tray -----------------------------------------------------------------------------------------------
TRAY = (0.10, 0.08, 0.03)  # outside (x, y, z) [m]
TRAY_WALL = 0.003

# -- scoring ---------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.004  # a seated cell's centre within this of the holder's centre (x, y) [m]
SEAT_Z_TOL = 0.003
SEAT_ALIGN = 0.94  # |cos| between the cell's axis and the holder's x (about 20 degrees)
