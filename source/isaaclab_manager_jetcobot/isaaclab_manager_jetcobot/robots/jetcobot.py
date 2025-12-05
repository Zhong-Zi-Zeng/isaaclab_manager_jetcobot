import isaaclab.sim as sim_utils  
from isaaclab.assets import ArticulationCfg  
from isaaclab.actuators import ImplicitActuatorCfg  

JETCOBOT_CFG = ArticulationCfg(  
    spawn=sim_utils.UsdFileCfg(  
        usd_path=f"./source/isaaclab_manager_jetcobot/isaaclab_manager_jetcobot/data/Robots/jetcobot.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Joint_1": 0.0,
            "Joint_2": -0.757,            
            "Joint_3": -0.678,
            "Joint_4": 0.0,
            "Joint_5": 0.0,
            "Joint_6": 0.0,
            "gripper_controller": -0.422,
        },
    ),
    actuators={  
        "arm_joints": ImplicitActuatorCfg(  
            joint_names_expr=["Joint_[1-6]"],  
            stiffness=None,
            damping=None,
        ),  
        "gripper": ImplicitActuatorCfg(  
            joint_names_expr=["gripper_controller"],  
            stiffness=None,
            damping=None,
        ),  
    }, 
)