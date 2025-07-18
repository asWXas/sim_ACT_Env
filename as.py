
# --------------------------- 创建实例 ------------------------------------------- #
import numpy as np
from isaacsim import SimulationApp
import keyword
# 是否  启用无头模式
print("是否启用无头模式？(True/False)")
headless = input()
if headless.lower() == "true":
    simulation_app = SimulationApp({"headless": True})
else:
    simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import Sdf, UsdLux
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
import matplotlib.pyplot as plt
import cv2 as cv

from ulits import *

# --------------------------------- Base Ground Plane --------------------------- #
GroundPlane(prim_path="/World/GroundPlane", z_position=0)

# --------------------------------- Lights --------------------------- ---------- #
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# --------------------------------- World ------------------------------------------- #
my_world = World(stage_units_in_meters=1.0)


# --------------------------------- add stage ---------------------------- ---------- #
add_value = stage_utils.add_reference_to_stage(usd_path="Env/robot/rmb.usd", prim_path="/World/env/robot")
articulation = Articulation(prim_paths_expr="/World/env/robot/rma/root_joint")

camera_r = Camera(
    prim_path="/World/camera_right",
    position=np.array([0.0, 0.0, 2.0]),
    frequency=30,
    resolution=(640, 480),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)
camera_l = Camera(
    prim_path="/World/camera_left",
    position=np.array([0.0, 0.0, 2.0]),
    frequency=30,
    resolution=(640, 480),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

camera_h = Camera(
    prim_path="/World/env/robot/rma/Link6/camera",
    frequency=30,
    resolution=(640, 480),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

# -------------------------------- reset and initialize physics ----------------------------- #
my_world.reset()
articulation.initialize()
camera_l.initialize()
camera_r.initialize()
camera_h.initialize()

camera_l.add_motion_vectors_to_frame()
camera_r.add_motion_vectors_to_frame()
camera_h.add_motion_vectors_to_frame()


joints =['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'finger_joint', 'left_outer_finger_joint', 'right_outer_finger_joint']
position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



# --------------------------------- 自己定义函数 ---------- #

camera_tool = CameraTool()
arm = RM65()



# my_world.add_render_callback(callback_name=callback_name, callback_fn=callback_fn)
# my_world.add_stage_callback(callback_name="test", callback_fn=test)
# my_world.add_physics_callback(callback_name=callback_name, callback_fn=callback_fn)
# my_world.add_timeline_callback(callback_name="test", callback_fn=test)



# 开始模拟循环
while simulation_app.is_running():
    my_world.step(render=True)
    img = camera_tool.get_image(camera_l)
    position = arm.get_joint_angles()
    articulation.set_joint_positions(positions=position, joint_names=joints)
# 自动关闭模拟应用
simulation_app.close()


