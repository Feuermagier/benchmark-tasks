"""The cabinet's USD, written with plain ``pxr`` after Isaac Sim has started: one fixed-base articulation.

Stage ``/Cabinet`` (the articulation root), every body authored with the door open at 90 degrees, level, and the
bottom arm on its plate (all joints at 0), in the cabinet frame of ``design.py``:
- ``carcass``: fixed to the world; the panels, both mounting plates and the clipped top hinge's arm.
- ``knuckle``: the top hinge's moving part; ``joints/swing`` turns it about the hinge axis (0: open 90 degrees,
  positive closes the door).
- ``door``: the door with its handle and the hinge cups; ``joints/sag`` turns it in its own plane about the top hinge
  (positive: its free edge droops).
- ``bottom_arm``: the bottom hinge's arm, on the door; ``joints/arm`` turns it about the hinge axis (0: on its plate
  with the door at 90 degrees, negative: its rear away from the side panel).
- ``bottom_tongue``: the bottom plate's sprung latch tongue, on the carcass; ``joints/tongue`` slides it out of the
  plate's face (0: flush, ``LATCH_PROUD``: at rest).
"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from hinge import design as d

OUT = Path(tempfile.gettempdir()) / "yam_tasks" / "cabinet_hinge"
FRICTION = (0.6, 0.5)  # melamine, steel
WHITE, DOOR_RGB, BACK_RGB = (0.93, 0.93, 0.91), (0.80, 0.70, 0.55), (0.85, 0.85, 0.83)
STEEL, NICKEL, FOOT = (0.62, 0.63, 0.65), (0.78, 0.79, 0.80), (0.12, 0.12, 0.12)


def linear_rgb(srgb) -> tuple[float, float, float]:
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb)


def _quat_x_to(v) -> Gf.Quatf:
    """The rotation taking +x to the direction ``v``."""
    v = np.asarray(v, float) / np.linalg.norm(v)
    x = np.array([1.0, 0.0, 0.0])
    axis = np.cross(x, v)
    s, c = np.linalg.norm(axis), float(np.dot(x, v))
    if s < 1e-9:
        return Gf.Quatf(1.0) if c > 0 else Gf.Quatf(0.0, 0.0, 0.0, 1.0)
    axis /= s
    half = math.atan2(s, c) / 2
    return Gf.Quatf(math.cos(half), *(axis * math.sin(half)).tolist())


class Stage:
    def __init__(self, root: str):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        self.root = root
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, root).GetPrim())
        UsdGeom.Scope.Define(self.stage, f"{root}/joints")
        self._mats: dict[str, UsdShade.Material] = {}
        self.grip = UsdShade.Material.Define(self.stage, f"{root}/Looks/grip")
        api = UsdPhysics.MaterialAPI.Apply(self.grip.GetPrim())
        api.CreateStaticFrictionAttr(FRICTION[0])
        api.CreateDynamicFrictionAttr(FRICTION[1])
        api.CreateRestitutionAttr(0.0)
        self.origins: dict[str, np.ndarray] = {}

    def material(self, rgb, metallic: float = 0.0) -> UsdShade.Material:
        key = "c" + "".join(f"{round(c * 255):02x}" for c in rgb) + f"m{round(metallic * 10)}"
        if key in self._mats:
            return self._mats[key]
        base = f"{self.root}/Looks/{key}"
        mat = UsdShade.Material.Define(self.stage, base)
        surf = UsdShade.Shader.Define(self.stage, f"{base}/surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4 if metallic else 0.6)
        surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(rgb)))
        mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
        self._mats[key] = mat
        return mat

    def body(self, name: str, origin, mass: float, inertia=None) -> str:
        path = f"{self.root}/{name}"
        xf = UsdGeom.Xform.Define(self.stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, origin)))
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*([float(inertia)] * 3)))
        self.origins[path] = np.asarray(origin, float)
        return path

    def _shape(self, prim, rgb, collide: bool, metallic: float) -> None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.material(rgb, metallic))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(self.grip, UsdShade.Tokens.weakerThanDescendants, "physics")

    def box(self, body: str, name: str, lo, hi, rgb, collide: bool = True, metallic: float = 0.0) -> None:
        """An axis-aligned box from corner ``lo`` to ``hi``, given in the cabinet frame."""
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        cube = UsdGeom.Cube.Define(self.stage, f"{body}/{name}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2 - self.origins[body]).tolist()))
        cube.AddScaleOp().Set(Gf.Vec3f(*(hi - lo).tolist()))
        self._shape(cube.GetPrim(), rgb, collide, metallic)

    def cylinder(self, body: str, name: str, axis: str, radius: float, centre, length: float, rgb,
                 collide: bool = True, metallic: float = 0.0) -> None:  # fmt: skip
        cyl = UsdGeom.Cylinder.Define(self.stage, f"{body}/{name}")
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(centre, float) - self.origins[body]).tolist()))
        self._shape(cyl.GetPrim(), rgb, collide, metallic)

    def revolute(self, name: str, body0: str, body1: str, at, axis, lower: float, upper: float) -> None:
        """About ``axis`` through ``at`` (cabinet frame), limits in rad."""
        rot = _quat_x_to(axis)
        at = np.asarray(at, float)
        j = UsdPhysics.RevoluteJoint.Define(self.stage, f"{self.root}/joints/{name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(at - self.origins[body0]).tolist()))
        j.CreateLocalRot0Attr(rot)
        j.CreateLocalPos1Attr(Gf.Vec3f(*(at - self.origins[body1]).tolist()))
        j.CreateLocalRot1Attr(rot)
        j.CreateLowerLimitAttr(math.degrees(lower))
        j.CreateUpperLimitAttr(math.degrees(upper))
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular").CreateTypeAttr("force")

    def prismatic(self, name: str, body0: str, body1: str, at, axis, lower: float, upper: float) -> None:
        """Along ``axis`` through ``at`` (cabinet frame), limits in m."""
        rot = _quat_x_to(axis)
        at = np.asarray(at, float)
        j = UsdPhysics.PrismaticJoint.Define(self.stage, f"{self.root}/joints/{name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(at - self.origins[body0]).tolist()))
        j.CreateLocalRot0Attr(rot)
        j.CreateLocalPos1Attr(Gf.Vec3f(*(at - self.origins[body1]).tolist()))
        j.CreateLocalRot1Attr(rot)
        j.CreateLowerLimitAttr(float(lower))
        j.CreateUpperLimitAttr(float(upper))
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear").CreateTypeAttr("force")

    def save(self, name: str) -> str:
        path = OUT / f"{name}.usda"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(path))
        return str(path)


def _plate(s: Stage, body: str, z: float) -> None:
    """A cruciform mounting plate on the left side panel's inner face, centred at height ``z``."""
    px, (pl, ph, pw) = d.PLATE_X, d.PLATE
    y0 = d.SIDE_IN_Y
    s.box(body, f"plate_{round(z * 1000)}", (px, y0 - ph, z - pw / 2), (px + pl, y0, z + pw / 2), NICKEL, metallic=0.8)
    wx, wt, wz = d.PLATE_WINGS
    xc = px + pl / 2
    s.box(body, f"wings_{round(z * 1000)}", (xc - wx / 2, y0 - wt, z - wz / 2), (xc + wx / 2, y0, z + wz / 2), NICKEL,
          collide=False, metallic=0.8)  # fmt: skip
    for k, dz in enumerate((-wz / 2 + 0.006, wz / 2 - 0.006)):  # the screw heads
        s.cylinder(body, f"screw_{round(z * 1000)}_{k}", "Y", 0.004, (xc, y0 - wt - 0.0008, z + dz), 0.0016, STEEL,
                   collide=False, metallic=0.9)  # fmt: skip


def _arm(s: Stage, body: str, name: str, z: float) -> None:
    """A hinge arm lying on its plate (cabinet frame, door open 90 degrees), with its crank round the panel's edge."""
    al, at, aw = d.ARM
    y_on = d.SIDE_IN_Y - d.PLATE[1]  # the plate's face
    s.box(body, f"{name}_arm", (d.ARM_FRONT_X, y_on - at, z - aw / 2), (d.ARM_FRONT_X + al, y_on, z + aw / 2), NICKEL,
          metallic=0.8)  # fmt: skip
    # The adjusting screw's head on the arm's rear half, and the crank out to the cup (visual).
    s.cylinder(body, f"{name}_screw", "Y", 0.0045, (d.ARM_FRONT_X + 0.072, y_on - at - 0.001, z), 0.002, STEEL,
               collide=False, metallic=0.9)  # fmt: skip
    s.box(body, f"{name}_crank", (d.AXIS_X - 0.016, y_on - at, z - aw / 2), (d.ARM_FRONT_X + 0.002, d.AXIS_Y - 0.004,
          z + aw / 2), NICKEL, collide=False, metallic=0.8)  # fmt: skip


def build_cabinet() -> str:
    s = Stage("/Cabinet")
    UsdPhysics.ArticulationRootAPI.Apply(s.stage.GetPrimAtPath("/Cabinet"))
    W, D, H, T = d.W, d.D, d.H, d.T
    carcass = s.body("carcass", (0.0, 0.0, 0.0), 6.0)
    fix = UsdPhysics.FixedJoint.Define(s.stage, "/Cabinet/joints/fix_base")
    fix.CreateBody1Rel().SetTargets([Sdf.Path(carcass)])
    s.box(carcass, "left", (0, W / 2 - T, 0), (D, W / 2, H), WHITE)
    s.box(carcass, "right", (0, -W / 2, 0), (D, -W / 2 + T, H), WHITE)
    s.box(carcass, "bottom", (0, -W / 2 + T, 0), (D - d.BACK_T, W / 2 - T, T), WHITE)
    s.box(carcass, "top", (0, -W / 2 + T, H - T), (D - d.BACK_T, W / 2 - T, H), WHITE)
    s.box(carcass, "back", (D - d.BACK_T, -W / 2 + T, T), (D, W / 2 - T, H - T), BACK_RGB)
    for k, (fx, fy) in enumerate(((0.04, 0.11), (0.04, -0.11), (D - 0.04, 0.11), (D - 0.04, -0.11))):
        s.cylinder(carcass, f"foot{k}", "Z", d.FOOT_R, (fx, fy, -d.FEET_H / 2), d.FEET_H, FOOT, collide=False)
    for z in (d.Z_TOP, d.Z_BOT):
        _plate(s, carcass, z)
    _arm(s, carcass, "top", d.Z_TOP)  # the top hinge stays clipped

    axis_top = (d.AXIS_X, d.AXIS_Y, d.Z_TOP)
    knuckle = s.body("knuckle", axis_top, d.KNUCKLE_MASS, inertia=1e-6)
    door = s.body("door", axis_top, d.DOOR_MASS + d.HANDLE_MASS)
    dw, dt, dh = d.DOOR
    edge = d.AXIS_X - 0.002  # the door's hinge edge (x), its inner face on the hinge axis
    s.box(door, "panel", (edge - dw, d.AXIS_Y, 0.0), (edge, d.AXIS_Y + dt, dh), DOOR_RGB)
    hx = edge - dw + d.HANDLE_FROM_EDGE
    hy = d.AXIS_Y + dt + d.HANDLE_STANDOFF - d.HANDLE_R
    hz = d.HANDLE_Z
    s.cylinder(door, "handle_bar", "Z", d.HANDLE_R, (hx, hy, hz), d.HANDLE_PITCH + 0.04, STEEL, metallic=0.9)
    for k, dz in enumerate((-d.HANDLE_PITCH / 2, d.HANDLE_PITCH / 2)):
        s.cylinder(door, f"handle_post{k}", "Y", 0.005, (hx, (d.AXIS_Y + dt + hy) / 2, hz + dz), hy - d.AXIS_Y - dt,
                   STEEL, metallic=0.9)  # fmt: skip
    cup_x = edge - d.CUP_FROM_EDGE
    for name, z in (("cup_top", d.Z_TOP), ("cup_bottom", d.Z_BOT)):
        s.cylinder(door, name, "Y", d.CUP_R + 0.006, (cup_x, d.AXIS_Y - 0.0007, z), 0.0014, NICKEL, collide=False,
                   metallic=0.8)  # fmt: skip
        s.box(door, f"{name}_boss", (cup_x - 0.008, d.AXIS_Y - 0.010, z - 0.009), (cup_x + 0.012, d.AXIS_Y, z + 0.009),
              NICKEL, collide=False, metallic=0.8)  # fmt: skip

    axis_bot = (d.AXIS_X, d.AXIS_Y, d.Z_BOT)
    arm = s.body("bottom_arm", axis_bot, d.ARM_MASS)
    _arm(s, arm, "bottom", d.Z_BOT)

    # The bottom plate's latch tongue (authored pressed flush; ``joints/tongue`` slides it out of the plate's face).
    y_on = d.SIDE_IN_Y - d.PLATE[1]
    tx, tt, tz = d.TONGUE
    t0 = (d.TONGUE_X + tx / 2, y_on, d.Z_BOT)
    tongue = s.body("bottom_tongue", t0, d.TONGUE_MASS, inertia=1e-7)
    s.box(tongue, "tongue", (d.TONGUE_X, y_on, d.Z_BOT - tz / 2), (d.TONGUE_X + tx, y_on + tt, d.Z_BOT + tz / 2), STEEL,
          metallic=0.9)  # fmt: skip
    s.prismatic("tongue", carcass, tongue, t0, (0, -1, 0), 0.0, d.LATCH_PROUD)

    s.revolute("swing", carcass, knuckle, axis_top, (0, 0, 1), d.SWING_OPEN, d.SWING_SHUT)
    s.revolute("sag", knuckle, door, axis_top, (0, -1, 0), d.SAG_MIN, d.SAG_MAX)
    s.revolute("arm", door, arm, axis_bot, (0, 0, 1), math.radians(-100), math.radians(25))
    return s.save("cabinet")
