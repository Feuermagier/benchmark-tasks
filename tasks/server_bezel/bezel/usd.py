"""The rack, server and bezel as one USD articulation, written with plain ``pxr`` after Isaac Sim has started.

Stage ``/Rack`` (the articulation root), authored with the bezel installed (all joints at 0), in the rack frame of
``design.py``:
- ``rack``: fixed to the world; the open-frame rack and the server with its rack ears and drive bays.
- ``free_x``, ``free_y``, ``free_z``, ``free_tilt``, ``free_roll``: massless-ish bodies that, with the joints of the same
  names (slides along x, y, z of the rack, then a turn about y and one about x, all at the bezel's hook pivot), let the
  bezel move freely; ``joints/swing`` turns the bezel about its hook pivot's vertical. Free, they have no drives; once
  the bezel is hooked their drives hold it there (and once latched, the swing's too).
- ``bezel``: the bezel.
"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from bezel import design as d

OUT = Path(tempfile.gettempdir()) / "yam_tasks" / "server_bezel"
FRICTION = (0.6, 0.5)  # plastic, painted steel
RACK_RGB, SERVER_RGB, BEZEL_RGB = (0.12, 0.12, 0.13), (0.55, 0.56, 0.58), (0.10, 0.10, 0.11)
GRILLE, BAY_RGB, ACCENT, LED = (0.22, 0.22, 0.24), (0.30, 0.30, 0.32), (0.70, 0.71, 0.73), (0.1, 0.8, 0.2)
FREE = ("free_x", "free_y", "free_z", "free_tilt", "free_roll")


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

    def joint(self, kind: str, name: str, body0: str, body1: str, at, axis: str, lower: float, upper: float) -> None:
        """A prismatic (m) or revolute (rad) joint along/about the frame's ``axis`` ("X", "Y", "Z") through ``at``."""
        at = np.asarray(at, float)
        cls = UsdPhysics.PrismaticJoint if kind == "prismatic" else UsdPhysics.RevoluteJoint
        j = cls.Define(self.stage, f"{self.root}/joints/{name}")
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(at - self.origins[body0]).tolist()))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(at - self.origins[body1]).tolist()))
        conv = (lambda v: float(v)) if kind == "prismatic" else math.degrees
        j.CreateLowerLimitAttr(conv(lower))
        j.CreateUpperLimitAttr(conv(upper))
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if kind == "prismatic" else "angular").CreateTypeAttr("force")

    def save(self, name: str) -> str:
        path = OUT / f"{name}.usda"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(path))
        return str(path)


def build_rack() -> str:
    s = Stage("/Rack")
    UsdPhysics.ArticulationRootAPI.Apply(s.stage.GetPrimAtPath("/Rack"))
    rack = s.body("rack", (0.0, 0.0, 0.0), 20.0)
    fix = UsdPhysics.FixedJoint.Define(s.stage, "/Rack/joints/fix_base")
    fix.CreateBody1Rel().SetTargets([Sdf.Path(rack)])
    # The open frame: base and top frames, four posts; the front posts carry the server's ears.
    px, py = d.POST
    fx = d.EAR[0]  # the front posts' front face is behind the ears
    for k, x0 in enumerate((fx, d.RACK_DEPTH - px)):
        for j, sy in enumerate((1, -1)):
            yc = sy * (d.POST_Y + py / 2 - 0.004)
            s.box(rack, f"post{k}{j}", (x0, yc - py / 2, 0.0), (x0 + px, yc + py / 2, d.RACK_H), RACK_RGB, metallic=0.3)
    for name, z0, z1 in (("base", 0.0, d.RACK_BASE), ("top", d.RACK_H - 0.015, d.RACK_H)):
        for sy in (1, -1):
            yc = sy * (d.POST_Y + py / 2 - 0.004)
            s.box(rack, f"{name}_side{(sy + 1) // 2}", (fx, yc - py / 2, z0), (d.RACK_DEPTH, yc + py / 2, z1), RACK_RGB, metallic=0.3)
        for k, x0 in enumerate((fx, d.RACK_DEPTH - px)):
            s.box(rack, f"{name}_cross{k}", (x0, -d.POST_Y, z0), (x0 + px, d.POST_Y, z1), RACK_RGB, metallic=0.3)
    # The server: chassis, rack ears, drive bays, power button and LED.
    sd, sw, sh = d.SERVER
    z0, z1 = d.SERVER_Z, d.SERVER_Z + sh
    s.box(rack, "chassis", (0.0, -sw / 2, z0), (sd, sw / 2, z1), SERVER_RGB, metallic=0.5)
    for sy in (1, -1):
        y_in, y_out = sy * sw / 2, sy * (sw / 2 + d.EAR[1])
        s.box(rack, f"ear{(sy + 1) // 2}", (0.0, min(y_in, y_out), z0), (d.EAR[0], max(y_in, y_out), z1), SERVER_RGB, metallic=0.5)
    # The right ear's two hook slots (dark, visual) and the left ear's latch catch.
    for k, dz in enumerate((-0.012, 0.012)):
        s.box(rack, f"slot{k}", (-0.0004, -sw / 2 - 0.004, d.SERVER_ZC + dz - 0.005),
              (0.0, -sw / 2 - 0.0005, d.SERVER_ZC + dz + 0.005), (0.02, 0.02, 0.02), collide=False)  # fmt: skip
    s.box(rack, "catch", (-0.0004, sw / 2 + 0.0005, d.SERVER_ZC - 0.008), (0.0, sw / 2 + 0.005, d.SERVER_ZC + 0.008),
          (0.02, 0.02, 0.02), collide=False)  # fmt: skip
    bw, bh = d.BAY
    for k in range(4):
        y0 = -sw / 2 + 0.03 + k * (bw + 0.004)
        s.box(rack, f"bay{k}", (-0.0004, y0, d.SERVER_ZC - bh / 2 - 0.006), (0.0, y0 + bw, d.SERVER_ZC + bh / 2 - 0.006),
              BAY_RGB, collide=False, metallic=0.4)  # fmt: skip
    s.box(rack, "led", (-0.0006, sw / 2 - 0.03, d.SERVER_ZC + 0.012), (0.0, sw / 2 - 0.024, d.SERVER_ZC + 0.016), LED,
          collide=False)  # fmt: skip

    # The bezel's free motion: a chain of joints at its hook pivot.
    hook = np.array(d.HOOK)
    prev = rack
    for name, kind, axis, lo, hi in (("free_x", "prismatic", "X", -0.5, 0.3), ("free_y", "prismatic", "Y", -0.3, 0.6),
                                     ("free_z", "prismatic", "Z", -0.25, 0.3),
                                     ("free_tilt", "revolute", "Y", -math.radians(120), math.radians(120)),
                                     ("free_roll", "revolute", "X", -math.radians(90), math.radians(90))):  # fmt: skip
        link = s.body(name, hook, d.CHAIN_MASS, inertia=1e-6)
        s.joint(kind, name, prev, link, hook, axis, lo, hi)
        prev = link
    bz = s.body("bezel", hook, d.BEZEL_MASS)
    s.joint("revolute", "swing", prev, bz, hook, "Z", -math.radians(10), math.radians(200))
    bd, bl, bh2 = d.BEZEL
    hx, hy, hz = hook
    s.box(bz, "shell", (hx - bd, hy, hz - bh2 / 2), (hx, hy + bl, hz + bh2 / 2), BEZEL_RGB)
    # Its front grille, a light accent strip, the key lock and latch release at its left end (visual).
    s.box(bz, "grille", (hx - bd - 0.0006, hy + 0.03, hz - bh2 / 2 + 0.006), (hx - bd, hy + bl - 0.06, hz + bh2 / 2 - 0.006),
          GRILLE, collide=False)  # fmt: skip
    s.box(bz, "accent", (hx - bd - 0.0008, hy + 0.01, hz - 0.002), (hx - bd, hy + 0.025, hz + 0.002), ACCENT,
          collide=False, metallic=0.6)  # fmt: skip
    s.cylinder(bz, "lock", "X", 0.007, (hx - bd - 0.001, hy + bl - 0.03, hz), 0.002, ACCENT, collide=False, metallic=0.8)
    s.box(bz, "release", (hx - bd + 0.004, hy + bl - 0.0005, hz - 0.006), (hx - 0.004, hy + bl + 0.0025, hz + 0.006),
          ACCENT, collide=False)  # fmt: skip
    tx, ty, tz = d.TAB
    for k, dz in enumerate((-0.012, 0.012)):
        s.box(bz, f"tab{k}", (hx, hy + 0.001, hz + dz - tz / 2), (hx + tx, hy + 0.001 + ty, hz + dz + tz / 2), BEZEL_RGB,
              collide=False)  # fmt: skip
    return s.save("rack")
