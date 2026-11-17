"""The clock in Isaac Lab-Arena: its scene object, its pose for scoring, and the episode reset (clock and old cell).

Import after Isaac Sim has started.
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
import numpy as np
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

from clock import design as d
from clock.usd import build_clock

SEATED_SPRING = -0.0035  # the spring's position with a cell seated (m)


class ClockObject(Object):
    """The clock as a floating-base Arena articulation; its spring is a position drive."""

    def __init__(self, name: str, pos, seated: bool = True):
        self.seated = seated  # read while the base class builds the config
        super().__init__(name=name, object_type=ObjectType.ARTICULATION, usd_path=build_clock(),
                         initial_pose=Pose(position_xyz=tuple(pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))  # fmt: skip

    def _generate_articulation_cfg(self) -> ArticulationCfg:
        cfg = ArticulationCfg(
            prim_path=self.prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.usd_path,
                rigid_props=PhysxRigidBodyPropertiesCfg(max_depenetration_velocity=0.5),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos={"spring": SEATED_SPRING if self.seated else 0.0},
                                                       joint_vel={".*": 0.0}),  # fmt: skip
            actuators={"spring": ImplicitActuatorCfg(joint_names_expr=["spring"], **d.SPRING_DRIVE)},
        )
        return self._add_initial_pose_to_cfg(cfg)


class Clock:
    """Runs the clock (env 0): its pose for scoring, the spring's preload, and the reset."""

    def __init__(self, env, name: str, old_cell: str):
        self.base = env.unwrapped
        self.device = self.base.device
        self.art = self.base.scene[name]
        self.old = self.base.scene[old_cell]
        ids, _ = self.art.find_joints(["spring"])
        self.spring = list(ids)
        self._preload()

    def _preload(self) -> None:
        v = torch.tensor([[d.SPRING_PRELOAD]], dtype=torch.float32, device=self.device)
        self.art.actuators.target_command.set_position_index(value=v, joint_ids=self.spring)

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        """The clock's frame in the env (position, quaternion xyzw)."""
        p = self.art.data.root_link_pose_w.torch[0].detach().cpu().numpy().copy()
        p[:3] -= self.base.scene.env_origins[0].detach().cpu().numpy()
        return p[:3], p[3:]

    def spring_pos(self) -> float:
        return float(self.art.data.joint_pos.torch[0, self.spring[0]])

    def reset(self) -> None:
        env_ids = torch.tensor([0], device=self.device)
        origin = self.base.scene.env_origins[:1]
        zero6 = torch.zeros(1, 6, device=self.device)
        for asset in (self.art, self.old):  # the old cell goes back in the holder, whatever the reset's jitter
            pose = asset.data.default_root_pose.torch[:1].clone()
            pose[:, :3] += origin
            asset.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
            asset.write_root_velocity_to_sim_index(root_velocity=zero6, env_ids=env_ids)
        q = torch.full((1, self.art.num_joints), SEATED_SPRING, device=self.device)
        self.art.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q), env_ids=env_ids)
        self._preload()
