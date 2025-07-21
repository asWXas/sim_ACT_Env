

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
GroundPlane(prim_path="/World/GroundPlane", z_position=0)

# --------------------------------- Lights --------------------------- ---------- #
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# --------------------------------- World ------------------------------------------- #
my_world = World(stage_units_in_meters=1.0)
single_xform_prim =SingleXFormPrim(
    prim_path="/World",
    name="World"
    )

# --------------------------------- add stage ---------------------------- ---------- #
add_value = stage_utils.add_reference_to_stage(usd_path="Env/robot/rmb.usd", prim_path="/World/env/robot")
add_value = stage_utils.add_reference_to_stage(usd_path="Env/asset/cabinc.usd", prim_path="/World/env/asset/cabinc")
articulation = Articulation(prim_paths_expr="/World/env/robot/rma/root_joint")
my_world.reset()
articulation.initialize()



# --------------------------------- XFormPrim ---------------------------- ---------- #

single_xform_prim = SingleXFormPrim(
    prim_path="/World/env/asset/cabinc",
    position=np.array([0.7903739690293761, 0, 0.3896903918659067]),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 180]), degrees=True),
    )

# --------------------------------- Camera ---------- ---------- #
camera_r = Camera(
    prim_path="/World/camera_right",
    position=np.array([-2.9103394, -3.118106 ,  1.6537194]),
    frequency=30,
    resolution=(640, 480),
    orientation=[ 0.92355239, -0.03532607,  0.11724859,  0.36339485],
)
camera_l = Camera(
    prim_path="/World/camera_left",
    position=np.array([-2.8797894,  3.4197335,  1.381893]),
    frequency=30,
    resolution=(640, 480),
    orientation=[ 0.91387818,  0.03837286,  0.08894618, -0.39426231],
)


camera_h = Camera(
    prim_path="/World/env/robot/rma/Link6/camera",
    frequency=30,
    resolution=(640, 480),
)
# --------------------------------- XFormPrim ---------------------------- ---------- #
   
camera_h.set_world_pose(
    position=[-1.1346314e+00,  2.1145139e-05,  8.7801039e-01],
    orientation=[ 9.97307397e-01, -2.47359278e-06, -7.33345454e-02, -9.14931304e-06]
)

# -------------------------------- reset and initialize physics ----------------------------- #

camera_l.initialize()
camera_r.initialize()
camera_h.initialize()

camera_l.add_motion_vectors_to_frame()
camera_r.add_motion_vectors_to_frame()
camera_h.add_motion_vectors_to_frame()


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
    img_l = camera_tool.get_image(camera_l)
    img_r = camera_tool.get_image(camera_r)
    img_h = camera_tool.get_image(camera_h)
    if img_l is not False:
        cv.imshow("left", img_l)
        cv.waitKey(1)
    if img_r is not False:
        cv.imshow("right", img_r)
        cv.waitKey(1)
    if img_h is not False:
        cv.imshow("head", img_h)
        cv.waitKey(1)
    position = arm.get_joint_angles()
    # 加入 3 个 0 值，代表保持姿态不动
    position = np.concatenate((position, [0, 0, 0,0.1,-0.1]))
    articulation.set_joint_positions(positions=position, joint_names=joints)
# 自动关闭模拟应用
simulation_app.close()


