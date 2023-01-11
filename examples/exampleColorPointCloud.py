import sys
import cv2
import numpy as np
import open3d as o3d

sys.path.insert(1, './')
import pykinect_azure as pykinect
from pykinect_azure.utils import Open3dVisualizer

if __name__ == "__main__":

	# Initialize the library, if the library is not found, add the library path as argument
	pykinect.initialize_libraries()

	# Modify camera configuration
	device_config = pykinect.default_configuration
	device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
	device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED

	# Start device
	device = pykinect.start_device(config=device_config)

	# Initialize the Open3d visualizer
	open3dVisualizer = Open3dVisualizer()

	cv2.namedWindow('Transformed color',cv2.WINDOW_NORMAL)
	while True:

		# Get capture
		capture = device.update()

		# Get the 3D point cloud
		ret, points = capture.get_pointcloud() 

		# Get the color image in the depth camera axis
		ret, color_image = capture.get_transformed_color_image()
        
		if not ret:
			continue

		# ret,color = capture.get_color_image()
		# print(ret,color)

		# pcd = o3d.geometry.PointCloud()
		# pcd.points = o3d.utility.Vector3dVector(points)
		# colors = cv2.cvtColor(color_image, cv2.COLOR_BGRA2RGB).reshape(-1,3)/255
		# pcd.colors = o3d.utility.Vector3dVector(colors)
		# pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
		# num = np.random.randint(100)
		# o3d.io.write_point_cloud('pointcloud'+str(num)+'.ply', pcd)
		# o3d.visualization.draw_geometries([pcd])

		open3dVisualizer(points, color_image)

		cv2.imshow('Transformed color', color_image)
		
		
		# Press q key to stop
		if cv2.waitKey(1) == ord('q'):  
			break
