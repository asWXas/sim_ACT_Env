# --------------------------- 创建实例 ------------------------------------------- #
import numpy as np
from isaacsim import SimulationApp
import keyword
# 是否  启用无头模式
print("是否启用无头模式？(Y/N)")
headless = input()
if headless.lower() == "Y":
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
from isaacsim.core.prims import SingleXFormPrim
import isaacsim.core.utils.numpy.rotations as rot_utils
import matplotlib.pyplot as plt
import cv2 as cv
from isaacsim.core.prims import XFormPrim

from ulits import *

# --------------------------------- Base Ground Plane --------------------------- #
# --------------------------------- World ------------------------------------------- #
my_world = World()
single_xform_prim = SingleXFormPrim(
    prim_path="/World/env",
    name="World",
    position=np.array([0, 0, 0]),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
    )
GroundPlane(prim_path="/World/GroundPlane", z_position=0)

stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/env/distantLight"))
distantLight.CreateIntensityAttr(300)

# --------------------------------- add stage ---------------------------- ---------- #
add_value = stage_utils.add_reference_to_stage(usd_path="Env/robot/rmb.usd", prim_path="/World/env/robot")
add_value = stage_utils.add_reference_to_stage(usd_path="Env/asset/cabinc.usd", prim_path="/World/env/asset/cabinc")
articulation = Articulation(prim_paths_expr="/World/env/robot/rma/root_joint")


# --------------------------------- XFormPrim ---------------------------- ---------- #
single_xform_prim = SingleXFormPrim(
    prim_path="/World/env/asset/cabinc",
    position=np.array([0.7903739690293761, 0, 0.3896903918659067]),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 180]), degrees=True),
    )

# --------------------------------- Camera ---------- ---------- #    
# 修复的相机代码 - 右侧相机
camera_r = Camera(
    prim_path="/World/env/camera_right",
    frequency=30,
    translation=np.array([0.1, 0, 0.3]),
    resolution=(640, 480),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
)

# 修复的相机代码 - 左侧相机
camera_l = Camera(
    prim_path="/World/env/camera_left",
    frequency=30,
    resolution=(640, 480),
)

# 头部相机
camera_h = Camera(
    prim_path="/World/env/robot/rma/Link6/camera",
    frequency=30,
    resolution=(640, 480),
)

# -------------------------------- reset and initialize physics ----------------------------- #
my_world.reset()

camera_l.initialize()
camera_r.initialize()
camera_h.initialize()

camera_l.add_motion_vectors_to_frame()
camera_r.add_motion_vectors_to_frame()
camera_h.add_motion_vectors_to_frame()


articulation.initialize()

print(articulation.dof_names)

joints =['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'finger_joint', 'left_outer_finger_joint', 'right_outer_finger_joint', 'left_inner_finger_joint', 'right_inner_finger_joint']
# --------------------------------- 自己定义函数 ---------- #
# 定义回调函数

arm = RM65()
camera_tool = CameraTool()
stage_callback = StageCallback(arm)

# my_world.add_render_callback(callback_name=callback_name, callback_fn=callback_fn)
my_world.add_stage_callback(callback_name="stage_callback", callback_fn=stage_callback.stage_callback)
# my_world.add_physics_callback(callback_name=callback_name, callback_fn=callback_fn)
# my_world.add_timeline_callback(callback_name="test", callback_fn=test)

# 开始模拟循环
while simulation_app.is_running():
    my_world.step(render=True)
    # img_l = camera_tool.get_image(camera_l)
    # img_r = camera_tool.get_image(camera_r)
    # img_h = camera_tool.get_image(camera_h)
    # if img_l is not False:
    #     cv.imshow("left", img_l)
    #     cv.waitKey(1)
    # if img_r is not False:
    #     cv.imshow("right", img_r)
    #     cv.waitKey(1)
    # if img_h is not False:
    #     cv.imshow("head", img_h)
    #     cv.waitKey(1)
    # position = [0,0,0,0,0,0]
    # # 加入 3 个 0 值，代表保持姿态不动
    # position = np.concatenate((position, [0, 0, 0,0.1,-0.1]))
    # articulation.set_joint_positions(positions=position, joint_names=joints)
# 自动关闭模拟应用
simulation_app.close()
