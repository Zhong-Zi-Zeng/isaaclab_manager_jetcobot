from isaaclab.utils import configclass  
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab_tasks.manager_based.manipulation.stack.config.franka.stack_joint_pos_env_cfg import FrankaCubeStackEnvCfg  
from isaaclab_tasks.manager_based.manipulation.stack import mdp
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg
from isaaclab.markers.config import FRAME_MARKER_CFG

from isaaclab_manager_jetcobot.robots import JETCOBOT_CFG 
  
@configclass  
class JetcobotLiftEnvCfg(LiftEnvCfg):  
    def __post_init__(self):  
        # post init of parent  
        super().__post_init__()
        
        # Setting object pose range and scale
        self.events.reset_object_position.params['pose_range'] = {"x": (-0.30, -0.25), "y": (-0.1, 0.1), "z": (0.0, 0.0)}        
        
        # Setting taget pose range
        self.commands.object_pose.ranges.pos_x = (0.1, 0.2)
        self.commands.object_pose.ranges.pos_y = (-0.1, 0.1)
        self.commands.object_pose.ranges.pos_z = (0.1, 0.15)        
        
        # Set Franka as robot
        self.scene.robot = JETCOBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["Joint_.*"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper_controller"],
            open_command_expr={"gripper_controller": 0.15},
            close_command_expr={"gripper_controller": -0.7},
        )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "Link_6"

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.5, 0.5, 0.5),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.05, 0.05, 0.05)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/jetcobot/base_link",
            debug_vis=True,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/jetcobot/Link_6",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.12374, 0.0, 0.01131],
                    ),
                ),
            ],
        )
  
        