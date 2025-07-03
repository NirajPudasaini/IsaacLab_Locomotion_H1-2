# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`UNITREE_A1_CFG`: Unitree A1 robot with DC motor model for the legs
* :obj:`UNITREE_GO1_CFG`: Unitree Go1 robot with actuator net model for the legs
* :obj:`UNITREE_GO2_CFG`: Unitree Go2 robot with DC motor model for the legs
* :obj:`H1_CFG`: H1 humanoid robot
* :obj:`H1_MINIMAL_CFG`: H1 humanoid robot with minimal collision bodies
* :obj:`G1_CFG`: G1 humanoid robot
* :obj:`G1_MINIMAL_CFG`: G1 humanoid robot with minimal collision bodies

Reference: https://github.com/unitreerobotics/unitree_ros
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
#from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

import os
base_dir = "/home/nyuair/niraj_projects/niraj_github/IsaacLab_Locomotion_H1-2"

###########################

H12_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path = os.path.join(base_dir, "robots", "h1_2_no_hand", "h1_2_no_hand.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.05), #same in gym
        joint_pos={
            ".*_hip_yaw.*": 0.0,
            ".*_hip_roll.*": 0.0,
            ".*_hip_pitch.*": -0.16,  # -16 degrees (original h1), -0.16 in gym
            ".*_knee.*": 0.36,  # 45 degrees (original h1) , 0.36 in gym
            ".*_ankle_pitch.*": -0.2,  # -30 degrees (original h1), -0.2 in gym for ankle pitch and 0 for roll
            ".*_ankle_roll.*": 0,  # -30 degrees (original h1), -0.2 in gym for ankle pitch and 0 for roll

            "torso.*": 0.0,

            ".*_shoulder_pitch.*": 0.4, # 0.4 in gym 
            ".*_shoulder_roll.*": 0.0,
            ".*_shoulder_yaw.*": 0.0,
            ".*_elbow_pitch.*": 0.3, #0.3 in gym
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,

    #Set everything from https://github.com/unitreerobotics/unitree_rl_gym/blob/main/legged_gym/envs/h1_2/h1_2_config.py
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw.*", ".*_hip_roll.*", ".*_hip_pitch.*", ".*_knee.*", "torso.*"],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*_hip_yaw.*": 200.0,
                ".*_hip_roll.*": 200.0,
                ".*_hip_pitch.*": 200.0,
                ".*_knee.*": 300.0,
                "torso.*": 200.0,
            },
            damping={
                ".*_hip_yaw.*": 2.5,
                ".*_hip_roll.*": 2.5,
                ".*_hip_pitch.*": 2.5,
                ".*_knee.*": 4.0,
                "torso.*": 5.0,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch.*", ".*_ankle_roll.*"],
            effort_limit=100,
            velocity_limit=100.0,
            stiffness={".*_ankle_pitch.*": 40.0,
                       ".*_ankle_roll.*": 40.0},
            damping={".*_ankle_pitch.*": 2.0,
                     ".*ankle_roll.*" : 2.0 },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch.*", ".*_shoulder_roll.*", ".*_shoulder_yaw.*", ".*_elbow_pitch.*"],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*_shoulder_pitch.*": 4000.0,
                ".*_shoulder_roll.*": 4000.0,
                ".*_shoulder_yaw.*": 4000.0,
                ".*_elbow_pitch.*": 4000.0,
            },
            damping={
                ".*_shoulder_pitch.*": 100.0,
                ".*_shoulder_roll.*": 100.0,
                ".*_shoulder_yaw.*": 100.0,
                ".*_elbow_pitch.*": 100.0,
            },
        ),
    },
)
"""Configuration for the Unitree H1 Humanoid robot."""


H12_MINIMAL_CFG = H12_CFG.copy()
H12_MINIMAL_CFG.spawn.usd_path = os.path.join(base_dir, "robots", "h1_2_no_hand", "h1_2_no_hand.usd")


"""Configuration for the Unitree H1 Humanoid robot with fewer collision meshes.

This configuration removes most collision meshes to speed up simulation.
"""

###########################

# H1_CFG = ArticulationCfg(
#     spawn=sim_utils.UsdFileCfg(
#         usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/H1/h1_minimal.usd",
#         activate_contact_sensors=True,
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(
#             disable_gravity=False,
#             retain_accelerations=False,
#             linear_damping=0.0,
#             angular_damping=0.0,
#             max_linear_velocity=1000.0,
#             max_angular_velocity=1000.0,
#             max_depenetration_velocity=1.0,
#         ),
#         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#             enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
#         ),
#     ),
#     init_state=ArticulationCfg.InitialStateCfg(
#         pos=(0.0, 0.0, 1.05),
#         joint_pos={
#             ".*_hip_yaw": 0.0,
#             ".*_hip_roll": 0.0,
#             ".*_hip_pitch": -0.28,  # -16 degrees
#             ".*_knee": 0.79,  # 45 degrees
#             ".*_ankle": -0.52,  # -30 degrees
#             "torso": 0.0,
#             ".*_shoulder_pitch": 0.28,
#             ".*_shoulder_roll": 0.0,
#             ".*_shoulder_yaw": 0.0,
#             ".*_elbow": 0.52,
#         },
#         joint_vel={".*": 0.0},
#     ),
#     soft_joint_pos_limit_factor=0.9,
#     actuators={
#         "legs": ImplicitActuatorCfg(
#             joint_names_expr=[".*_hip_yaw", ".*_hip_roll", ".*_hip_pitch", ".*_knee", "torso"],
#             effort_limit=300,
#             velocity_limit=100.0,
#             stiffness={
#                 ".*_hip_yaw": 150.0,
#                 ".*_hip_roll": 150.0,
#                 ".*_hip_pitch": 200.0,
#                 ".*_knee": 200.0,
#                 "torso": 200.0,
#             },
#             damping={
#                 ".*_hip_yaw": 5.0,
#                 ".*_hip_roll": 5.0,
#                 ".*_hip_pitch": 5.0,
#                 ".*_knee": 5.0,
#                 "torso": 5.0,
#             },
#         ),
#         "feet": ImplicitActuatorCfg(
#             joint_names_expr=[".*_ankle"],
#             effort_limit=100,
#             velocity_limit=100.0,
#             stiffness={".*_ankle": 20.0},
#             damping={".*_ankle": 4.0},
#         ),
#         "arms": ImplicitActuatorCfg(
#             joint_names_expr=[".*_shoulder_pitch", ".*_shoulder_roll", ".*_shoulder_yaw", ".*_elbow"],
#             effort_limit=300,
#             velocity_limit=100.0,
#             stiffness={
#                 ".*_shoulder_pitch": 40.0,
#                 ".*_shoulder_roll": 40.0,
#                 ".*_shoulder_yaw": 40.0,
#                 ".*_elbow": 40.0,
#             },
#             damping={
#                 ".*_shoulder_pitch": 10.0,
#                 ".*_shoulder_roll": 10.0,
#                 ".*_shoulder_yaw": 10.0,
#                 ".*_elbow": 10.0,
#             },
#         ),
#     },
# )
# """Configuration for the Unitree H1 Humanoid robot."""


# H1_MINIMAL_CFG = H1_CFG.copy()
# H1_MINIMAL_CFG.spawn.usd_path = f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/H1/h1_minimal.usd"
# """Configuration for the Unitree H1 Humanoid robot with fewer collision meshes.

# This configuration removes most collision meshes to speed up simulation.
# """