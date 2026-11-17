"""Tilted and sideways grips for the scripted demo. NumPy only.

The adapter's waypoints (``yam_arena.demo.Waypoint``) only turn a straight-down grip about the vertical (``yaw``). The
demo here also points the gripper forward into the cabinet. :func:`grip` returns a float that carries a full
rotation; :func:`install` wraps ``yam_arena.demo.grip_rotation`` (in this process, without changing the adapter's
files) so that a waypoint whose ``yaw`` is such a value gets that rotation, and every other waypoint is unchanged.
"""

from __future__ import annotations

import numpy as np


class Grip(float):
    """A waypoint's ``yaw`` carrying the TCP rotation (columns: finger axis, third axis, approach)."""

    rot: np.ndarray

    def __new__(cls, rot: np.ndarray):
        g = super().__new__(cls, 0.0)
        g.rot = np.asarray(rot, float)
        return g


def grip(approach, fingers) -> Grip:
    """The gripper pointing along ``approach`` with its fingers closing along ``fingers`` (world vectors)."""
    z = np.asarray(approach, float)
    z /= np.linalg.norm(z)
    x = np.asarray(fingers, float)
    x = x - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return Grip(np.column_stack([x, np.cross(z, x), z]))


def install() -> None:
    import yam_arena.demo as demo

    if getattr(demo.grip_rotation, "_takes_grip", False):
        return
    original = demo.grip_rotation

    def grip_rotation(yaw: float = 0.0) -> np.ndarray:
        return yaw.rot if isinstance(yaw, Grip) else original(yaw)

    grip_rotation._takes_grip = True
    demo.grip_rotation = grip_rotation
