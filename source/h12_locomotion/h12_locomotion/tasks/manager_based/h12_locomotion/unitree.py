# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""Configuration for Unitree robots."""


#for armatures: https://github.com/correlllab/h12-lab-docs/blob/main/docs/specs.md

import os

# Prefer a repository-relative path so the USD assets bundled with this repo are used.
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

#from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
# from unitree_actuators import (
#     UnitreeActuatorCfg_N7520_14p3,
#     UnitreeActuatorCfg_N7520_22p5,
#     UnitreeActuatorCfg_N5020_16,
#     UnitreeActuatorCfg_M107_15,
#     UnitreeActuatorCfg_M107_24,
# )

#SET ARMATURES ~ can be found in unitree_actuators.py
UnitreeActuatorCfg_N7520_14p3 = 0.01017752
UnitreeActuatorCfg_N7520_22p5 = 0.025101925
UnitreeActuatorCfg_N5020_16 = 0.003609725
UnitreeActuatorCfg_M107_15 = 0.063259741
UnitreeActuatorCfg_M107_24 = 0.160478022

import math
NATURAL_FREQ = 10 * 2.0 * math.pi  # 10Hz
DAMPING_RATIO = 2.0

# Calculate Stiffness (Kp) for each motor type
STIFFNESS_M107_24 = UnitreeActuatorCfg_M107_24 * (NATURAL_FREQ**2)
STIFFNESS_M107_15 = UnitreeActuatorCfg_M107_15 * (NATURAL_FREQ**2)
STIFFNESS_N7520_22p5 = UnitreeActuatorCfg_N7520_22p5 * (NATURAL_FREQ**2)
STIFFNESS_N5020_16 = UnitreeActuatorCfg_N5020_16 * (NATURAL_FREQ**2)
STIFFNESS_N7520_14p3 = UnitreeActuatorCfg_N7520_14p3 * (NATURAL_FREQ**2)

# Calculate Damping (Kd) for each motor type
DAMPING_M107_24 = 2.0 * DAMPING_RATIO * UnitreeActuatorCfg_M107_24 * NATURAL_FREQ
DAMPING_M107_15 = 2.0 * DAMPING_RATIO * UnitreeActuatorCfg_M107_15 * NATURAL_FREQ
DAMPING_N7520_22p5 = 2.0 * DAMPING_RATIO * UnitreeActuatorCfg_N7520_22p5 * NATURAL_FREQ
DAMPING_N5020_16 = 2.0 * DAMPING_RATIO * UnitreeActuatorCfg_N5020_16 * NATURAL_FREQ
DAMPING_N7520_14p3 = 2.0 * DAMPING_RATIO * UnitreeActuatorCfg_N7520_14p3 * NATURAL_FREQ

print(f"STIFFNESS_M107_24: {STIFFNESS_M107_24}, DAMPING_M107_24: {DAMPING_M107_24}")
print(f"STIFFNESS_M107_15: {STIFFNESS_M107_15}, DAMPING_M107_15: {DAMPING_M107_15}")
print(f"STIFFNESS_N7520_22p5: {STIFFNESS_N7520_22p5}, DAMPING_N7520_22p5: {DAMPING_N7520_22p5}")
print(f"STIFFNESS_N5020_16: {STIFFNESS_N5020_16}, DAMPING_N5020_16: {DAMPING_N5020_16}")
print(f"STIFFNESS_N7520_14p3: {STIFFNESS_N7520_14p3}, DAMPING_N7520_14p3: {DAMPING_N7520_14p3}")

# exit()

H12_CFG_HANDLESS = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # Use the local USD asset bundled in this package (unitree_model/usd/...)
        # usd_path= "/home/niraj/isaac_projects/H12_Bullet_Time/h12_bullet_time/source/h12_bullet_time/h12_bullet_time/assets/robots/unitree_model/usd/h1_2_handless/h1_2_handless.usd",
        fix_base = False,
        replace_cylinders_with_capsules=True,

        asset_path= "/home/niraj/isaac_projects/H12_Bullet_Time/h12_bullet_time/source/h12_bullet_time/h12_bullet_time/assets/robots/gentact_descriptions/robots/h1-2/h1_2_handless.urdf",
       # asset_path= "/home/niraj/isaac_projects/H12_Bullet_Time/h12_bullet_time/source/h12_bullet_time/h12_bullet_time/assets/robots/unitree_model/urdf/h1_2/h1_2_simplified.urdf",

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
            enabled_self_collisions=False, 
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.05),               #same as gym
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.16, #same as gym
            "left_knee_joint": 0.36,    #same as gym
            "left_ankle_pitch_joint": -0.15, #gym is -0.2,reduced a bit to avoid foot collision with ground
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.16,
            "right_knee_joint": 0.36,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,

            "torso_joint": 0.0,

            # arms joints
            "left_shoulder_pitch_joint": 0.4, # ~ 23 degrees, same as gym
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.3,           # ~ 14.1 degrees, same as gym
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.4,  # ~ 23 degrees, same as gym
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.3,        # ~ 14.1 degrees, same as gym
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,


    # actuators={
    #     "legs": ImplicitActuatorCfg(
    #         joint_names_expr=[
    #             ".*_hip_yaw_joint", 
    #             ".*_hip_roll_joint",
    #             ".*_hip_pitch_joint", 
    #             ".*_knee_joint",
    #         ],
    #         effort_limit=300,
    #         velocity_limit=100,
    #         stiffness=None,
    #         damping=None,
    #         armature=None,
    #     ),
    #     "feet": ImplicitActuatorCfg(
    #         effort_limit=None,
    #         joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
    #         stiffness=None,
    #         damping=None,
    #         # armature=0.001,
    #     ),
    #     "arms": ImplicitActuatorCfg(
    #         joint_names_expr=[
    #             ".*_shoulder_.*_joint",
    #             ".*_elbow_joint",
    #             ".*_wrist_.*_joint"
    #         ],
    #         effort_limit=None,
    #         velocity_limit=None,
    #          stiffness={  # increase the stiffness (kp)
    #              ".*_shoulder_.*_joint": 25.0,
    #              ".*_elbow_joint": 50.0,
    #              ".*_wrist_.*_joint": 40.0,
    #         },
    #          damping={    # increase the damping (kd)
    #              ".*_shoulder_.*_joint": 2.0,
    #              ".*_elbow_joint": 2.0,
    #              ".*_wrist_.*_joint": 2.0,
    #          },
    #         armature=None,
    #     ),

actuators={
    # Motor: M107-24, Torque: 300 Nm
    # From your original "legs" group
    "hip_pitch_roll_knee": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_hip_pitch_joint", 
            ".*_hip_roll_joint",
            ".*_knee_joint"
        ],
        effort_limit_sim=300,
        velocity_limit_sim=100,
        stiffness={
            ".*_hip_pitch_joint": STIFFNESS_M107_24,
            ".*_hip_roll_joint": STIFFNESS_M107_24,
            ".*_knee_joint": STIFFNESS_M107_24,
        },
        damping={
            ".*_hip_pitch_joint": DAMPING_M107_24,
            ".*_hip_roll_joint": DAMPING_M107_24,
            ".*_knee_joint": DAMPING_M107_24,
        },
        armature=UnitreeActuatorCfg_M107_24,
    ),
    # Motor: M107-15, Torque: 200 Nm
    # From your original "legs" group
    "hip_yaw": ImplicitActuatorCfg(
        joint_names_expr=[".*_hip_yaw_joint"],
        effort_limit_sim=200,
        velocity_limit_sim=100,
        stiffness={"^.*_hip_yaw_joint$": STIFFNESS_M107_15},
        damping={"^.*_hip_yaw_joint$": DAMPING_M107_15},
        armature=UnitreeActuatorCfg_M107_15,
    ),
    # Motor: N7520-22.5, Torque: 120 Nm
    # From your original "arms" group
    "shoulder_pitch_roll_and_elbow": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_shoulder_pitch_joint",
            ".*_shoulder_roll_joint",
            ".*_elbow_joint"
        ],
        effort_limit_sim= 120,
        velocity_limit_sim= 100,
        stiffness={
            ".*_shoulder_.*_joint":STIFFNESS_N7520_22p5,
            ".*_elbow_joint": STIFFNESS_N7520_22p5,
        },
        damping={
            ".*_shoulder_.*_joint": DAMPING_N7520_22p5,
            ".*_elbow_joint": DAMPING_N7520_22p5,
        },
        armature=UnitreeActuatorCfg_N7520_22p5,
    ),
    # Motor: N5020-16, Torque: 25 Nm
    # From your original "arms" group
    "wrists": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_wrist_.*_joint"
        ],
        effort_limit_sim=25.0,
        velocity_limit_sim=100,
        stiffness={
            ".*_wrist_.*_joint": STIFFNESS_N5020_16,
        },
        damping={
            ".*_wrist_.*_joint": DAMPING_N5020_16,
        },
        armature=UnitreeActuatorCfg_N5020_16,
    ),
    # Motor: N7520-14.3, Torque: 75 Nm
    # Combined from your "feet" and "arms" groups
    "ankles_and_shoulder_yaw": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_ankle_pitch_joint",
            ".*_ankle_roll_joint",
            ".*_shoulder_yaw_joint"
        ],
        effort_limit_sim=75,
        velocity_limit_sim=100,
        stiffness={
            ".*_ankle_pitch_joint": STIFFNESS_N7520_14p3,
            ".*_ankle_roll_joint": STIFFNESS_N7520_14p3,
            ".*_shoulder_yaw_joint": STIFFNESS_N7520_14p3,
        },
        damping={
            ".*_ankle_pitch_joint": DAMPING_N7520_14p3,       # From original "feet" config
            ".*_ankle_roll_joint": DAMPING_N7520_14p3,       # From original "feet" config
            ".*_shoulder_yaw_joint": DAMPING_N7520_14p3,   # From original "arms" config
        },
        armature=UnitreeActuatorCfg_N7520_14p3,
    ),
    },
)

# H12_ACTION_SCALE = {}

# for actuator in H12_CFG_HANDLESS.actuators.values():
#     # Get effort and stiffness parameters
#     effort = actuator.effort_limit
#     stiffness = actuator.stiffness
#     names = actuator.joint_names_expr

#     # Ensure effort and stiffness are dicts for consistent access
#     if not isinstance(effort, dict):
#         effort = {n: effort for n in names}
#     if not isinstance(stiffness, dict):
#         stiffness = {n: stiffness for n in names}

#     # Compute scaling per joint
#     for n in names:
#         if n in effort and n in stiffness and stiffness[n]:
#             # scale = 0.25 * torque_limit / stiffness  (radians per normalized action unit)
#             H12_ACTION_SCALE[n] = 0.25 * effort[n] / stiffness[n]

# # Print summary for verification
# print("\n=== Computed H12_ACTION_SCALE values ===")
# for joint, scale in H12_ACTION_SCALE.items():
#     print(f"{joint}: {scale:.3f}")
H12_MINIMAL_CFG = H12_CFG_HANDLESS.copy()
# H12_MINIMAL_CFG.spawn.usd_path = os.path.join(base_dir, "h1_2_handless", "h1_2_handless.usd")


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