"""The cabinet in Isaac Lab-Arena: its scene object, the bottom hinge's clip, and the episode reset.

The clip: once the door is held level enough that the bottom arm's hook meets its plate's front lug
(``CLIP_SAG``) and, with the door held still (``CLIP_STILL``), the arm is pressed flat onto the plate (``CLIP_ANGLE``),
pressing the plate's sprung latch tongue down to ``LATCH_CLICK`` (``CLICK_FORCE``), the arm's latch snaps over the
plate's tail. From then on the hinge holds: the door's droop is held at level and the arm stays on its plate while the door
swings (its drive follows the door, so in the cabinet's frame it does not move; it is held a hair off the plate's face,
``CLIPPED_CLEAR``, so that following the door one control step late it does not grind into the plate).

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

from hinge import design as d
from hinge.usd import build_cabinet

JOINTS = ("swing", "sag", "arm", "tongue")
START = {"swing": 0.0, "sag": d.SAG_MAX, "arm": d.ARM_OFF, "tongue": d.LATCH_PROUD}


class CabinetObject(Object):
    """The cabinet as a fixed-base Arena articulation."""

    def __init__(self, name: str, pos):
        super().__init__(name=name, object_type=ObjectType.ARTICULATION, usd_path=build_cabinet(),
                         initial_pose=Pose(position_xyz=tuple(pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))  # fmt: skip

    def _generate_articulation_cfg(self) -> ArticulationCfg:
        cfg = ArticulationCfg(
            prim_path=self.prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.usd_path,
                rigid_props=PhysxRigidBodyPropertiesCfg(max_depenetration_velocity=0.5),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,  # the loose arm lands on its plate
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=dict(START), joint_vel={".*": 0.0}),
            actuators={
                "swing": ImplicitActuatorCfg(joint_names_expr=["swing"], stiffness=0.0, damping=d.SWING_DAMPING),
                "sag": ImplicitActuatorCfg(joint_names_expr=["sag"], **d.SAG_DRIVE),
                "arm": ImplicitActuatorCfg(joint_names_expr=["arm"], **d.ARM_DRIVE),
                "tongue": ImplicitActuatorCfg(joint_names_expr=["tongue"], **d.TONGUE_DRIVE),
            },
        )
        return self._add_initial_pose_to_cfg(cfg)


class Cabinet:
    """Runs the cabinet (env 0): the bottom hinge's clip and the reset."""

    def __init__(self, env, name: str):
        self.base = env.unwrapped
        self.device = self.base.device
        self.art = self.base.scene[name]
        ids, _ = self.art.find_joints(list(JOINTS), preserve_order=True)
        self.ids = dict(zip(JOINTS, ids, strict=True))
        self.clipped = False
        self._unclipped()

    def joints(self) -> dict[str, float]:
        q = self.art.data.joint_pos.torch[0].detach().cpu()
        return {n: float(q[i]) for n, i in self.ids.items()}

    def swing_speed(self) -> float:
        return abs(float(self.art.data.joint_vel.torch[0, self.ids["swing"]]))

    def _target(self, joint: str, value: float) -> None:
        v = torch.tensor([[value]], dtype=torch.float32, device=self.device)
        self.art.actuators.target_command.set_position_index(value=v, joint_ids=[self.ids[joint]])

    def _gains(self, joint: str, gains: dict) -> None:
        j = [self.ids[joint]]
        self.art.write_joint_stiffness_to_sim_index(stiffness=gains["stiffness"], joint_ids=j)
        self.art.write_joint_damping_to_sim_index(damping=gains["damping"], joint_ids=j)

    def _unclipped(self) -> None:
        self._target("tongue", d.LATCH_PROUD)
        self._gains("sag", d.SAG_DRIVE)
        self._gains("arm", d.ARM_DRIVE)
        self._target("sag", 0.0)
        self._target("arm", d.ARM_OFF)

    def after_step(self) -> str | None:
        """Snap the clip when the arm meets its plate; keep a clipped arm on its plate. Returns an event."""
        q = self.joints()
        if self.clipped:
            self._target("arm", -q["swing"] - d.CLIPPED_CLEAR)
            return None
        pressed = q["tongue"] < d.LATCH_CLICK and abs(q["swing"] + q["arm"]) < d.CLIP_ANGLE  # the latch pressed down
        if pressed and abs(q["sag"]) < d.CLIP_SAG and self.swing_speed() < d.CLIP_STILL:
            self.clipped = True
            self._gains("sag", d.CLIPPED_DRIVE)
            self._gains("arm", d.CLIPPED_DRIVE)
            self._target("sag", 0.0)
            self._target("arm", -q["swing"] - d.CLIPPED_CLEAR)
            self._target("tongue", 0.0)  # under the arm's latch
            return "clipped"
        return None

    def reset(self) -> None:
        env_ids = torch.tensor([0], device=self.device)
        q = torch.zeros(1, self.art.num_joints, device=self.device)
        for n, i in self.ids.items():
            q[0, i] = START[n]
        self.art.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q), env_ids=env_ids)
        self.clipped = False
        self._unclipped()
