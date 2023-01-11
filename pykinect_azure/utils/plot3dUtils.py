import numpy as np
import cv2
import open3d as o3d

class Open3dVisualizer():

	def __init__(self):

		self.point_cloud = o3d.geometry.PointCloud()
		self.o3d_started = False

		self.vis = o3d.visualization.VisualizerWithKeyCallback()
		self.vis.create_window()

	def __call__(self, points_3d, rgb_image=None):

		self.update(points_3d, rgb_image)

		return False

	def update(self, points_3d, rgb_image=None):
		coord_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.06, origin = [0,0,0])
		# coord_pcd = coord_mesh.compute_point_cloud()
		# Add values to vectors
		self.point_cloud.points = o3d.utility.Vector3dVector(points_3d)
		if rgb_image is not None:
			# colors = cv2.cvtColor(rgb_image, cv2.COLOR_BGRA2RGB).reshape(-1,3)/255
			# self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
			self.point_cloud.colors = o3d.utility.Vector3dVector(rgb_image)
		# self.point_cloud = points_3d

		self.point_cloud.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
		
		# Add geometries if it is the first time
		if not self.o3d_started:
			self.vis.add_geometry(self.point_cloud)
			self.vis.add_geometry(coord_mesh)
			self.o3d_started = True

		else:
			self.vis.update_geometry(self.point_cloud)
			self.vis.update_geometry(coord_mesh)

		self.vis.poll_events()
		self.vis.update_renderer()