import numpy as np  
from isaacsim.sensors.camera import Camera
import yaml
class LowPassFilter:
    def __init__(self, alpha, num_joints=6):
        self.alpha = alpha
        self.prev_values = np.zeros(num_joints)  # 初始化前一值

    def filter(self, new_values):
        self.prev_values = self.alpha * np.array(new_values) + (1 - self.alpha) * self.prev_values
        return self.prev_values.tolist()


class  CameraTool:
    def __init__(self):
        print("初始化相机工具")
    def get_image(self, camera: Camera):
        img = camera.get_rgb()
        if img is None or img.size == 0:
            print("前两帧可能为空！.....")
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

    def camera_config(self, camera: Camera, config: str | None = None):
        if isinstance(config, str) and (config.endswith('.yaml') or config.endswith('.yml')):  # 检查config是否为字符串并且以.yaml或.yml结尾
            try:
                with open(config, 'r') as f:  # 尝试以只读模式打开config文件
                    config = f.read()  # 读取文件内容并赋值给config
            except Exception as e:  # 捕获并处理可能的异常
                print(f"Error reading config file {config}: {e}")  # 打印错误信息
                return  # 返回，结束函数执行

        # 定义默认参数
        width, height = 1920, 1200  # 图像的宽度和高度，默认值为1920x1200
        camera_matrix = [[958.8, 0.0, 957.8],  # 相机矩阵，默认值
                        [0.0, 956.7, 589.5],
                        [0.0, 0.0, 1.0]]
        pixel_size = 3  # 像素大小，默认值为3微米
        f_stop = 1.8  # 光圈值，默认值为f/1.8
        focus_distance = 0.006  # 对焦距离，默认值为0.006米

        if config is not None:  # 如果config不为空
            try:
                config_data = yaml.safe_load(config)  # 使用yaml库解析config内容
                width = config_data.get('width', width)  # 从config_data中获取宽度，如果未指定则使用默认值
                height = config_data.get('height', height)  # 从config_data中获取高度，如果未指定则使用默认值
                camera_matrix = config_data.get('camera_matrix', camera_matrix)  # 从config_data中获取相机矩阵，如果未指定则使用默认值
                pixel_size = config_data.get('pixel_size', pixel_size)  # 从config_data中获取像素大小，如果未指定则使用默认值
                f_stop = config_data.get('f_stop', f_stop)  # 从config_data中获取光圈值，如果未指定则使用默认值
                focus_distance = config_data.get('focus_distance', focus_distance)  # 从config_data中获取对焦距离，如果未指定则使用默认值
            except Exception as e:  # 捕获并处理可能的异常
                print(f"Error parsing YAML config: {e}")  # 打印错误信息

        # 计算参数
        ((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix  # 从相机矩阵中提取fx和fy
        horizontal_aperture = width * pixel_size * 1e-3  # 计算水平视场角，单位为毫米
        vertical_aperture = height * pixel_size * 1e-3  # 计算垂直视场角，单位为毫米
        focal_length_x = fx * pixel_size * 1e-3  # 计算x方向的焦距，单位为毫米
        focal_length_y = fy * pixel_size * 1e-3  # 计算y方向的焦距，单位为毫米
        focal_length = (focal_length_x + focal_length_y) / 2  # 计算平均焦距，单位为毫米

        # 设置相机参数（将需要的单位转换为米）
        camera.set_focal_length(focal_length / 1e3)  # 设置焦距，单位为米
        camera.set_focus_distance(focus_distance)  # 设置对焦距离，单位为米
        camera.set_lens_aperture(f_stop * 1e3)  # 设置光圈值，单位为毫米
        camera.set_horizontal_aperture(horizontal_aperture / 1e3)  # 设置水平视场角，单位为米
        camera.set_vertical_aperture(vertical_aperture / 1e3)  # 设置垂直视场角，单位为米
        camera.set_clipping_range(0.05, 1.0e5)  # 设置裁剪范围，单位为米
