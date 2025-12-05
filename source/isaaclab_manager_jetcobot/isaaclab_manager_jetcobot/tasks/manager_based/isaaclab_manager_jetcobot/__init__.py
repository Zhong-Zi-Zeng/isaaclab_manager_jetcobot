# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

# Reach Task
gym.register(
    id="Isaaclab-Manager-Jetcobot-Reach-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaaclab_manager_jetcobot_reach_env_cfg:JetcobotReachEnvCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:reach_task_skrl_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaaclab-Manager-Jetcobot-Reach-v0-PLAY",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaaclab_manager_jetcobot_reach_env_cfg:JetcobotReachSceneCfg_PLAY",
        "skrl_cfg_entry_point": f"{agents.__name__}:reach_task_skrl_ppo_cfg.yaml",
    },
)

# Lift Task
gym.register(
    id="Isaaclab-Manager-Jetcobot-Lift-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaaclab_manager_jetcobot_lift_env_cfg:JetcobotLiftEnvCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:lift_task_skrl_ppo_cfg.yaml",
    },
)