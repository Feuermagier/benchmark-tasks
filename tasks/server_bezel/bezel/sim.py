"""The rack and bezel in Isaac Lab-Arena: the scene object, the bezel's hook and latch, and the episode reset.

The bezel moves freely (its chain of joints has no drives) until its hook tabs go into the right ear's slots: its
hook pivot within ``HOOK_TOL`` of its place, upright and square to the server within ``HOOK_TILT``, its left end
swung out between ``HOOK_MIN`` and ``HOOK_OPEN``. Then the tabs hold it there and it can only swing about them;
swung shut to within ``LATCH``, its latch clicks into the left ear and holds it shut.

Import after Isaac Sim has started.
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
import torch
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_arena.assets.object import Object
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import (
    PhysxArticulationRootPropertiesCfg,
    PhysxCollisionPropertiesCfg,
    PhysxRigidBodyPropertiesCfg,
)

from bezel import design as d
from bezel.usd import FREE, build_rack

JOINTS = (*FREE, "swing")
LINEAR = ("free_x", "free_y", "free_z")


class RackObject(Object):
    """The rack as a fixed-base Arena articulation; ``start`` gives the bezel's starting joint positions."""

    def __init__(self, name: str, pos, start: dict[str, float]):
        self.start = dict(start)  # read while the base class builds the config
        super().__init__(name=name, object_type=ObjectType.ARTICULATION, usd_path=build_rack(),
                         initial_pose=Pose(position_xyz=tuple(pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))  # fmt: skip

    def _generate_articulation_cfg(self) -> ArticulationCfg:
        lin, ang = d.CHAIN_DAMPING["lin"], d.CHAIN_DAMPING["ang"]
        cfg = ArticulationCfg(
            prim_path=self.prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.usd_path,
                rigid_props=PhysxRigidBodyPropertiesCfg(max_depenetration_velocity=0.5),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,  # the bezel against the server and rack
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=dict(self.start), joint_vel={".*": 0.0}),
            actuators={
                "linear": ImplicitActuatorCfg(joint_names_expr=list(LINEAR), stiffness=0.0, damping=lin),
                "angular": ImplicitActuatorCfg(joint_names_expr=["free_tilt", "free_roll", "swing"], stiffness=0.0,
                                               damping=ang),  # fmt: skip
            },
        )
        return self._add_initial_pose_to_cfg(cfg)


class Rack:
    """Runs the bezel (env 0): its hook, its latch and the reset."""

    def __init__(self, env, name: str, start: dict[str, float]):
        self.base = env.unwrapped
        self.device = self.base.device
        self.art = self.base.scene[name]
        self.start = dict(start)
        ids, _ = self.art.find_joints(list(JOINTS), preserve_order=True)
        self.ids = dict(zip(JOINTS, ids, strict=True))
        self.hooked = False
        self.latched = False
        self._free(JOINTS)

    def joints(self) -> dict[str, float]:
        q = self.art.data.joint_pos.torch[0].detach().cpu()
        return {n: float(q[i]) for n, i in self.ids.items()}

    def speed(self) -> float:
        v = self.art.data.joint_vel.torch[0].detach().cpu()
        return max(abs(float(v[self.ids[n]])) for n in LINEAR)

    def _set(self, names, gains: dict | None, targets: dict[str, float] | None = None) -> None:
        j = [self.ids[n] for n in names]
        for n in names:
            kind = "lin" if n in LINEAR else "ang"
            g = gains[kind] if gains else {"stiffness": 0.0, "damping": d.CHAIN_DAMPING[kind]}
            self.art.write_joint_stiffness_to_sim_index(stiffness=g["stiffness"], joint_ids=[self.ids[n]])
            self.art.write_joint_damping_to_sim_index(damping=g["damping"], joint_ids=[self.ids[n]])
        v = torch.tensor([[(targets or {}).get(n, 0.0) for n in names]], dtype=torch.float32, device=self.device)
        self.art.actuators.target_command.set_position_index(value=v, joint_ids=j)

    def _free(self, names) -> None:
        self._set(names, None)

    def after_step(self) -> str | None:
        """Hook and latch the bezel when it gets there. Returns an event."""
        q = self.joints()
        if not self.hooked:
            at = max(abs(q[n]) for n in LINEAR) < d.HOOK_TOL
            square = abs(q["free_tilt"]) < d.HOOK_TILT and abs(q["free_roll"]) < d.HOOK_TILT
            if at and square and d.HOOK_MIN < q["swing"] < d.HOOK_OPEN:
                self.hooked = True
                self._set(FREE, d.HELD)  # the tabs in their slots: held at its pivot, square to the server
                return "hooked"
        elif not self.latched and q["swing"] < d.LATCH:
            self.latched = True
            self._set(["swing"], d.HELD)
            return "latched"
        return None

    def reset(self) -> None:
        env_ids = torch.tensor([0], device=self.device)
        q = torch.zeros(1, self.art.num_joints, device=self.device)
        for n, i in self.ids.items():
            q[0, i] = self.start.get(n, 0.0)
        self.art.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q), env_ids=env_ids)
        self.hooked = self.latched = False
        self._free(JOINTS)
