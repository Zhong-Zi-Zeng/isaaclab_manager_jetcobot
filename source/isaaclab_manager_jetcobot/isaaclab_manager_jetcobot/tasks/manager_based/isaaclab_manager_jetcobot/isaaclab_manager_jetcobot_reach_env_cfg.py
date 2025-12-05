# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

from . import mdp

##
# Pre-defined configs
##

from isaaclab_manager_jetcobot.robots import JETCOBOT_CFG 


##
# Environment configuration
##

@configclass
class JetcobotReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # change number of envs
        self.scene.num_envs = 100

        # switch robot to jetcobot
        self.scene.robot = JETCOBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["Link_6"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["Link_6"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["Link_6"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["Joint_.*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose.body_name = "Link_6"
        self.commands.ee_pose.ranges.pos_x = (0.15, 0.25)
        self.commands.ee_pose.ranges.pos_y = (-0.1, 0.1)
        self.commands.ee_pose.ranges.pos_z = (0.15, 0.25)
        self.commands.ee_pose.ranges.pitch = (math.pi/2., math.pi/2.)        


@configclass
class JetcobotReachSceneCfg_PLAY(JetcobotReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False