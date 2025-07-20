from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
import cv2 as cv

# -----------------------------------  callback -------------------------------

# ------------------------------- timeline_callback

class TimelineCallback:
    def __init__(self, world):
        self.world = world
    def time_callback(self, event):
        pass



# --------------------------------- render_callback
class RenderCallback:
    def __init__(self, world):
        self.world = world
    def render_callback(self, event):
        pass

# ----------------------------------- physics_callback

class PhysicsCallback:
    def __init__(self, world):
        self.world = world
    def physics_callback(self, event):
        pass


# ----------------------------------- stage_callback

class StageCallback:
    def __init__(self, world):
        self.world = world
    def stage_callback(self, event):
        pass


# ----------------------------------- Tools and utilities -------------------------------------- #



