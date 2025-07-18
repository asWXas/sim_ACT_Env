import numpy as np  
from isaacsim.sensors.camera import Camera
class LowPassFilter:
    def __init__(self, alpha, num_joints=6):
        self.alpha = alpha
        self.prev_values = np.zeros(num_joints)  # 初始化前一值

    def filter(self, new_values):
        self.prev_values = self.alpha * np.array(new_values) + (1 - self.alpha) * self.prev_values
        return self.prev_values.tolist()


class  CameraTool:
    def __init__(self):
        pass
    def get_image(camera:Camera):
        img = camera.get_rgb()
        if img is None or img.size == 0:
            print("图像为空，无法保存")
            return False
        try:
            if img.dtype != np.uint8:
                img = (img.copy() * 255).astype(np.uint8)
            else:
                img = img.copy()
            return img
        except Exception as e:
            print(f"保存图像时出错: {e}")
            return False
