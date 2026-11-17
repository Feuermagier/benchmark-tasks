"""USD assets, written with plain ``pxr`` and PIL after Isaac Sim has started: the clock (an articulation: the case
with its movement and battery holder, and the holder's spring), the AA cells (rigid bodies) and the used-battery tray
(fixed).

The clock's stage (default prim ``/Clock``, the articulation root, a floating base):
- ``/Clock/case``: the root body, origin at the centre of the rim it lies on; the round case, the movement on its back
  and the holder's fixed parts (side ribs, the + contact block, the spring's backing block), with printed marks.
- ``/Clock/spring_link``: the - contact spring, a plate with a chamfered top, sliding along x
  (``/Clock/joints/spring``, 0 at rest, negative = compressed).
"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from clock import design as d

FRICTION = (0.8, 0.6)  # static, dynamic: a cell's printed wrap, the pack
CLOCK_FRICTION = (0.3, 0.25)  # the clock's moulded plastic and the holder's metal contacts
FONTS = Path(__file__).resolve().parent.parent / "assets" / "fonts"
OUT = Path(tempfile.gettempdir()) / "yam_tasks" / "clock_battery"
PX_PER_M = 8000
CASE, BLACK, METAL = (0.42, 0.28, 0.16), (0.08, 0.08, 0.09), (0.75, 0.75, 0.72)


def linear_rgb(srgb) -> tuple[float, float, float]:
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb)


class Stage:
    """A new stage with materials and primitive helpers in a body's frame."""

    def __init__(self, root: str, friction=FRICTION):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        self.root = root
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, root).GetPrim())
        self._mats: dict[str, UsdShade.Material] = {}
        self.grip = UsdShade.Material.Define(self.stage, f"{root}/Looks/grip")
        api = UsdPhysics.MaterialAPI.Apply(self.grip.GetPrim())
        api.CreateStaticFrictionAttr(friction[0])
        api.CreateDynamicFrictionAttr(friction[1])
        api.CreateRestitutionAttr(0.0)

    def material(self, rgb=(1, 1, 1), texture: str | None = None, metallic: float = 0.0) -> UsdShade.Material:
        key = (texture or "c" + "".join(f"{round(c * 255):02x}" for c in rgb) + f"m{round(metallic * 10)}")
        key = key.replace(".", "_")
        if key in self._mats:
            return self._mats[key]
        base = f"{self.root}/Looks/{key}"
        mat = UsdShade.Material.Define(self.stage, base)
        surf = UsdShade.Shader.Define(self.stage, f"{base}/surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        if texture:
            reader = UsdShade.Shader.Define(self.stage, f"{base}/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tex = UsdShade.Shader.Define(self.stage, f"{base}/image")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{texture}")
            tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))  # fmt: skip
        else:
            surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(rgb)))
        mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
        self._mats[key] = mat
        return mat

    def body(self, path: str, pos, mass: float) -> str:
        xf = UsdGeom.Xform.Define(self.stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(float(mass))
        return path

    def _collide(self, prim) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(self.grip, UsdShade.Tokens.weakerThanDescendants, "physics")

    def box(self, parent: str, name: str, size, pos, rgb, collide: bool = True, metallic: float = 0.0) -> None:
        cube = UsdGeom.Cube.Define(self.stage, f"{parent}/{name}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(self.material(rgb, metallic=metallic))
        if collide:
            self._collide(cube.GetPrim())

    def cylinder_x(self, parent: str, name: str, radius: float, length: float, x: float, rgb, collide: bool = True,
                   metallic: float = 0.0) -> None:  # fmt: skip
        """A cylinder along x, centred at ``x`` on the x axis."""
        cyl = UsdGeom.Cylinder.Define(self.stage, f"{parent}/{name}")
        cyl.CreateAxisAttr("X")
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.AddTranslateOp().Set(Gf.Vec3d(x, 0, 0))
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(self.material(rgb, metallic=metallic))
        if collide:
            self._collide(cyl.GetPrim())

    def hull(self, parent: str, name: str, points, rgb) -> None:
        """A convex solid through ``points``, collided as their convex hull."""
        from scipy.spatial import ConvexHull

        pts = np.asarray(points, float)
        centre = pts.mean(axis=0)
        faces = []
        for tri in ConvexHull(pts).simplices:  # wind each face outwards
            a, b, c = pts[tri]
            faces.append(tri if np.dot(np.cross(b - a, c - a), a - centre) > 0 else tri[::-1])
        mesh = UsdGeom.Mesh.Define(self.stage, f"{parent}/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts.tolist()])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr([int(i) for f in faces for i in f])
        mesh.CreateSubdivisionSchemeAttr("none")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(self.material(rgb))
        self._collide(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    def print_quad(self, parent: str, name: str, size_xy, centre, texture: str) -> None:
        """A printed rectangle facing up: image up along +x, image right along -y. Visual only."""
        sx, sy = size_xy
        cx, cy, z = centre
        mesh = UsdGeom.Mesh.Define(self.stage, f"{parent}/{name}")
        mesh.CreatePointsAttr([(cx - sx / 2, cy + sy / 2, z), (cx - sx / 2, cy - sy / 2, z),
                               (cx + sx / 2, cy - sy / 2, z), (cx + sx / 2, cy + sy / 2, z)])  # fmt: skip
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateSubdivisionSchemeAttr("none")
        primvars = UsdGeom.PrimvarsAPI(mesh)
        st = primvars.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(self.material(texture=texture))

    def save(self, name: str) -> str:
        path = OUT / f"{name}.usda"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(path))
        return str(path)


def _font(size: int):
    from PIL import ImageFont

    return ImageFont.truetype(str(FONTS / "DejaVuSans-Bold.ttf"), size)


def movement_image():
    """The movement's top, round the holder: + at the contact end (+x, image top), - at the spring end."""
    from PIL import Image, ImageDraw

    w, h = round(d.MOVEMENT[1] * PX_PER_M), round(d.MOVEMENT[0] * PX_PER_M)
    img = Image.new("RGB", (w, h), tuple(round(c * 255) for c in BLACK))
    draw = ImageDraw.Draw(img)
    white = (235, 235, 235)
    for y_img, mark in ((24, "+"), (h - 30, "−")):  # image top is the + end
        draw.text((w * 0.17, y_img), mark, font=_font(44), fill=white, anchor="mm")
        draw.text((w * 0.83, y_img), mark, font=_font(44), fill=white, anchor="mm")
    draw.text((w * 0.14, h / 2), "1.5V", font=_font(20), fill=white, anchor="mm")
    draw.text((w * 0.86, h / 2), "AA", font=_font(22), fill=white, anchor="mm")
    return img


def tray_image():
    from PIL import Image, ImageDraw

    inner = d.TRAY[0] - 2 * d.TRAY_WALL, d.TRAY[1] - 2 * d.TRAY_WALL
    w, h = round(inner[1] * 4000), round(inner[0] * 4000)
    img = Image.new("RGB", (w, h), (40, 110, 60))
    draw = ImageDraw.Draw(img)
    draw.text((w / 2, h * 0.38), "USED", font=_font(44), fill=(255, 255, 255), anchor="mm")
    draw.text((w / 2, h * 0.62), "BATTERIES", font=_font(36), fill=(255, 255, 255), anchor="mm")
    return img


def build_clock() -> str:
    s = Stage("/Clock", CLOCK_FRICTION)
    UsdPhysics.ArticulationRootAPI.Apply(s.stage.GetPrimAtPath("/Clock"))
    UsdGeom.Scope.Define(s.stage, "/Clock/joints")
    case = s.body("/Clock/case", (0, 0, 0), d.CLOCK_MASS)
    ring = [(d.CLOCK_R * math.cos(a), d.CLOCK_R * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, d.CLOCK_SIDES, endpoint=False)]  # fmt: skip
    s.hull(case, "shell", [(x, y, z) for x, y in ring for z in (0.0, d.CLOCK_DEPTH)], CASE)
    mx, my, mz = d.MOVEMENT
    s.box(case, "movement", d.MOVEMENT, (0, 0, d.CLOCK_DEPTH + mz / 2), BLACK)
    OUT.mkdir(parents=True, exist_ok=True)
    movement_image().save(OUT / "movement.png")
    s.print_quad(case, "marks", (mx, my), (0, 0, d.FLOOR_Z + 0.0002), "movement.png")
    f = d.FLOOR_Z
    _saddles(s, case, f, along_y=False)
    ex, ey, ez = d.END_WALL
    s.box(case, "plus_block", d.END_WALL, (d.PLUS_FACE_X + ex / 2, 0, f + ez / 2), BLACK)
    s.box(case, "plus_contact", (0.0008, 0.008, 0.008), (d.PLUS_FACE_X + 0.0004, 0, f + d.CELL_Z), METAL,
          collide=False, metallic=1.0)  # fmt: skip
    back = d.SPRING_REST_X - d.SPRING_TRAVEL - d.SPRING_T - 0.0005  # the spring's backing block's face
    s.box(case, "spring_block", d.END_WALL, (back - ex / 2, 0, f + ez / 2), BLACK)
    # The spring: a plate whose face (towards the cell) is upright for 5 mm, then chamfered back to its top.
    link = s.body("/Clock/spring_link", (d.SPRING_REST_X, 0, f), d.SPRING_MASS)
    t, w = d.SPRING_T, 0.006
    profile = ((0, 0), (-t, 0), (-t, d.SPRING_H), (0, d.SPRING_FACE_H))  # x, z: upright face, then the chamfer
    s.hull(link, "plate", [(x, y, z) for y in (-w, w) for x, z in profile], METAL)
    j = UsdPhysics.PrismaticJoint.Define(s.stage, "/Clock/joints/spring")
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(case)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(link)])
    j.CreateLocalPos0Attr(Gf.Vec3f(d.SPRING_REST_X, 0, f))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    j.CreateLowerLimitAttr(-d.SPRING_TRAVEL)
    j.CreateUpperLimitAttr(0.0)
    UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear").CreateTypeAttr("force")
    return s.save("clock")


def _saddles(s: Stage, parent: str, z0: float, along_y: bool) -> None:
    """The two saddles a cell rests on, centred on the parent's origin at height ``z0``, the cell along x (or y): each
    a block with a lip along either side of its top, so the cell cannot roll off."""
    sx, sy, sz = d.SADDLE
    lx, ly, lz = d.SADDLE_LIP
    rgb = (0.85, 0.88, 0.92) if along_y else BLACK
    for k, u in enumerate((-d.SADDLE_X, d.SADDLE_X)):
        for name, size, (a, b), z in ((f"saddle_{k}", (sx, sy, sz), (u, 0.0), z0 + sz / 2),
                                      (f"lip_{k}_a", (lx, ly, lz), (u, sy / 2 - ly / 2), z0 + sz + lz / 2),
                                      (f"lip_{k}_b", (lx, ly, lz), (u, ly / 2 - sy / 2), z0 + sz + lz / 2)):
            if along_y:
                size, a, b = (size[1], size[0], size[2]), b, a
            s.box(parent, name, size, (a, b, z), rgb)


def build_pack() -> str:
    """The new cell's opened blister pack, fixed: a printed card with two clear saddles, the cell along y."""
    s = Stage("/Pack")
    X, Y, Z = d.PACK_CARD
    s.box("/Pack", "card", (X, Y, Z), (0, 0, Z / 2), (0.92, 0.55, 0.15))
    _saddles(s, "/Pack", Z, along_y=True)
    return s.save("pack")


def build_battery(new: bool) -> str:
    """An AA cell as one rigid body along x, + terminal at +x. Origin at the can's centre (0.75 mm behind the cell's
    centre, the nub's half height)."""
    s = Stage("/Cell")
    prim = s.stage.GetPrimAtPath("/Cell")
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(d.AA_MASS)
    body, band = ((0.10, 0.10, 0.11), (0.85, 0.55, 0.20)) if new else ((0.45, 0.50, 0.55), (0.62, 0.64, 0.66))
    L = d.AA_BODY
    s.cylinder_x("/Cell", "can", d.AA_R, L - 0.014, -0.007, body)
    s.cylinder_x("/Cell", "band", d.AA_R, 0.014, L / 2 - 0.007, band, metallic=0.3)  # the + end's coloured band
    s.cylinder_x("/Cell", "nub", d.AA_NUB[0], d.AA_NUB[1], L / 2 + d.AA_NUB[1] / 2, METAL, metallic=1.0)
    return s.save("cell_new" if new else "cell_old")


def build_tray() -> str:
    """The used-battery tray: a floor and four walls, fixed. Origin at the centre of its underside."""
    s = Stage("/Tray")
    X, Y, Z = d.TRAY
    t = d.TRAY_WALL
    green = (0.16, 0.43, 0.24)
    s.box("/Tray", "floor", (X, Y, t), (0, 0, t / 2), green)
    walls = (("w_far", (t, Y, Z), (X / 2 - t / 2, 0, Z / 2)), ("w_near", (t, Y, Z), (-X / 2 + t / 2, 0, Z / 2)),
             ("w_left", (X, t, Z), (0, Y / 2 - t / 2, Z / 2)), ("w_right", (X, t, Z), (0, -Y / 2 + t / 2, Z / 2)))
    for name, size, pos in walls:
        s.box("/Tray", name, size, pos, green)
    OUT.mkdir(parents=True, exist_ok=True)
    tray_image().save(OUT / "tray.png")
    s.print_quad("/Tray", "label", (X - 2 * t, Y - 2 * t), (0, 0, t + 0.0002), "tray.png")
    return s.save("tray")
