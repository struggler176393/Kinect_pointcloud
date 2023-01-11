

import sys
import cv2
import numpy as np
import open3d as o3d

sys.path.insert(1, '../')
import pykinect_azure as pykinect
from pykinect_azure.utils import Open3dVisualizer
import argparse


class SaveWithCallback:

    def __init__(self, config, device, align_depth_to_color):
        self.flag_exit = False
        self.align_depth_to_color = align_depth_to_color

        self.sensor = o3d.io.AzureKinectSensor(config)
        if not self.sensor.connect(device):
            raise RuntimeError('Failed to connect to sensor')

    def escape_callback(self, vis):
        self.flag_exit = True
        return False
    
    def save_callback(self, vis):
        num = np.random.randint(100)
        o3d.io.write_image('color'+str(num)+'.png', self.rgbd.color, quality=- 1)
        o3d.io.write_image('depth'+str(num)+'.png', self.rgbd.depth, quality=- 1)
        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_save = 83
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_save, self.save_callback)
        vis.create_window('viewer', 1920, 540)
        print("Sensor initialized. Press [ESC] to exit.")

        vis_geometry_added = False
        while not self.flag_exit:
            self.rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            print(self.align_depth_to_color)
            if self.rgbd is None:
                continue

            if not vis_geometry_added:
                vis.add_geometry(self.rgbd)
                vis_geometry_added = True
            
            
            # vis.capture_screen_image('b.png', do_render=False)
            vis.update_geometry(self.rgbd)
            vis.poll_events()
            vis.update_renderer()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
    parser.add_argument('--config', type=str, default='myconfig.json', help='input json kinect config')
    parser.add_argument('--list',
                        action='store_true',
                        help='list available azure kinect sensors')
    parser.add_argument('--device',
                        type=int,
                        default=0,
                        help='input kinect device id')
    parser.add_argument('-a',
                        '--align_depth_to_color',
                        action='store_false',
                        help='enable align depth image to color')
    args = parser.parse_args()

    if args.list:
        o3d.io.AzureKinectSensor.list_devices()
        exit()

    if args.config is not None:
        config = o3d.io.read_azure_kinect_sensor_config(args.config)
    else:
        config = o3d.io.AzureKinectSensorConfig()

    device = args.device
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0

    v = SaveWithCallback(config, device, args.align_depth_to_color)
    v.run()



