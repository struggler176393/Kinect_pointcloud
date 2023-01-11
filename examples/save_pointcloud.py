import sys
sys.path.insert(1, './')
import argparse
import open3d as o3d
import numpy as np
import pykinect_azure as pykinect
from pykinect_azure.utils import Open3dVisualizer
import cv2
from datetime import datetime

class SaverWithCallback:

    def __init__(self, config, device, align_depth_to_color):
        self.flag_exit = False
        self.align_depth_to_color = align_depth_to_color
        self.open3dVisualizer = Open3dVisualizer()

        self.sensor = o3d.io.AzureKinectSensor(config)
        if not self.sensor.connect(device):
            raise RuntimeError('Failed to connect to sensor')

    def escape_callback(self, vis):
        self.flag_exit = True
        return False
    
    def save_callback(self, vis):
        timeStr = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        o3d.io.write_image('color_'+timeStr+'.png', self.rgbd.color, quality=- 1)
        o3d.io.write_image('depth_'+timeStr+'.png', self.rgbd.depth, quality=- 1)
        o3d.io.write_point_cloud('point_cloud_'+timeStr+'.ply', self.point_cloud)
        return False

    def run(self):
        while not self.flag_exit:
            glfw_key_escape = 256
            glfw_key_save = 83
            self.open3dVisualizer.vis.register_key_callback(glfw_key_escape, self.escape_callback)
            self.open3dVisualizer.vis.register_key_callback(glfw_key_save, self.save_callback)

            self.rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            if self.rgbd is None:
                continue
            color_image = self.rgbd.color
            depth_image = self.rgbd.depth

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image, depth_image,
                convert_rgb_to_intensity=False)
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault )
            intrinsic.set_intrinsics(
                width=512, height=512, fx=251.730835, fy=251.734344, cx=259.099640, cy=262.528076)

            self.point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

            self.open3dVisualizer(self.point_cloud.points,self.point_cloud.colors)
            cv2.imshow('Transformed color', cv2.cvtColor(np.asarray(color_image), cv2.COLOR_BGRA2RGB))
            
            # Press q key to stop
            if cv2.waitKey(1) == ord('q'):  
                break


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
    parser.add_argument('--config', type=str, default='./myconfig.json', help='input json kinect config')
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

    v = SaverWithCallback(config, device, args.align_depth_to_color)
    v.run()
