import numpy as np
import sapien
from transforms3d.euler import euler2quat

from mani_skill import PACKAGE_ASSET_DIR
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent


@register_agent()
class OpenArmBi(BaseAgent):
    """
    Dual-arm robot: each arm has 7 revolute joints + 2 prismatic finger joints.
    Total DoF = 18.
    """

    uid = "openarm_bi"

    # ----------------------------
    # URDF
    # ----------------------------
    urdf_path = f"{PACKAGE_ASSET_DIR}/robots/openarm_bi/openarm_bi.urdf"
    urdf_config = dict()

    # ----------------------------
    # Joint names extracted from your URDF
    # ----------------------------

    # 左臂 7 DOF
    left_arm_joint_names = [
        "openarm_left_joint1",
        "openarm_left_joint2",
        "openarm_left_joint3",
        "openarm_left_joint4",
        "openarm_left_joint5",
        "openarm_left_joint6",
        "openarm_left_joint7",
    ]

    # 左手两个手指
    left_gripper_joint_names = [
        "openarm_left_finger_joint1",
        "openarm_left_finger_joint2",
    ]

    # 右臂 7 DOF
    right_arm_joint_names = [
        "openarm_right_joint1",
        "openarm_right_joint2",
        "openarm_right_joint3",
        "openarm_right_joint4",
        "openarm_right_joint5",
        "openarm_right_joint6",
        "openarm_right_joint7",
    ]

    # 右手两个手指
    right_gripper_joint_names = [
        "openarm_right_finger_joint1",
        "openarm_right_finger_joint2",
    ]

    @property
    def all_joint_names(self):
        """按顺序拼接全部 18 个关节"""
        return (
            self.left_arm_joint_names
            + self.left_gripper_joint_names
            + self.right_arm_joint_names
            + self.right_gripper_joint_names
        )

    # ----------------------------
    # Keyframes
    # ----------------------------
    keyframes = dict(
        zero=Keyframe(
            qpos=np.zeros(18),
            pose=sapien.Pose(q=euler2quat(0, 0, 0)),
        )
    )

    # ----------------------------
    # Controller Configurations
    # ----------------------------
    @property
    def _controller_configs(self):
        joint_names = self.all_joint_names

        # PD 位置控制
        pd_joint_pos = PDJointPosControllerConfig(
            joint_names,
            lower=None,
            upper=None,
            stiffness=[1e3] * len(joint_names),
            damping=[2e2] * len(joint_names),
            force_limit=200,
            normalize_action=False,
        )

        # PD Δ位置控制（有助于 teleop 防抖）
        pd_joint_delta_pos = PDJointPosControllerConfig(
            joint_names,
            lower=[-0.05] * len(joint_names),
            upper=[0.05] * len(joint_names),
            stiffness=[1e3] * len(joint_names),
            damping=[2e2] * len(joint_names),
            force_limit=200,
            use_delta=True,
        )

        return dict(
            pd_joint_pos=pd_joint_pos,
            pd_joint_delta_pos=pd_joint_delta_pos,
        )

    # ----------------------------
    # Post-load hook
    # ----------------------------
    def _after_loading_articulation(self):
        """如果你需要绑定 TCP，或记录 tip link，可以在这里做"""
        super()._after_loading_articulation()

        # 示例（如果你的 URDF 有这些 link）
        # self.left_tcp = self.robot.links_map["openarm_left_hand_tcp_link"]
        # self.right_tcp = self.robot.links_map["openarm_right_hand_tcp_link"]
        pass

    # ----------------------------
    # TCP for viewer
    # ----------------------------
    @property
    def tcp_pos(self):
        """返回左右手末端的平均位置（只是 viewer 需要）"""
        try:
            left = self.robot.links_map["openarm_left_joint7_link"].pose.p
            right = self.robot.links_map["openarm_right_joint7_link"].pose.p
            return (left + right) / 2
        except:
            return np.zeros(3)

