from pxr import Usd
import os
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

# Construct USD path using nucleus directory
usd_path = f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/H1/h1_minimal.usd"

# Check if the USD file exists
if not os.path.exists(usd_path):
    raise FileNotFoundError(f"USD file does not exist: {usd_path}")

# Open the stage
stage = Usd.Stage.Open(usd_path)
if not stage:
    raise RuntimeError(f"Failed to open USD file: {usd_path}")

print(f"Inspecting USD file: {usd_path}\n")

# List all prims under /World/envs/env_0/Robot (adjust if necessary)
robot_root = "/World/envs/env_0/Robot"
robot_prim = stage.GetPrimAtPath(robot_root)
if not robot_prim:
    raise RuntimeError(f"Robot prim not found at {robot_root}")

print("--- All Prims Under Robot Root ---")
for prim in stage.TraverseAll():
    if prim.GetPath().pathString.startswith(robot_root):
        print(prim.GetPath().pathString)

# Filter for link and joint prims
print("\n--- Link Prims (names ending with _link) ---")
for prim in stage.TraverseAll():
    name = prim.GetName()
    if name.endswith("_link"):
        print(f"{prim.GetPath()} -> {name}")

print("\n--- Joint Prims (names ending with _joint) ---")
for prim in stage.TraverseAll():
    name = prim.GetName()
    if name.endswith("_joint"):
        print(f"{prim.GetPath()} -> {name}")

# Optionally, inspect articulation attributes
print("\n--- Articulation Attributes ---")
for prim in stage.TraverseAll():
    if prim.GetTypeName() == "PhysicsRigidBodyAPI":
        print(f"RigidBody: {prim.GetPath()}")
    if prim.GetTypeName() == "PhysicsArticulationRootAPI":
        print(f"Articulation Root: {prim.GetPath()}")
