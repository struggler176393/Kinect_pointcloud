import pyk4a
import open3d as o3d

# 初始化相机
k4a = pyk4a.Device()
k4a.start()

# 获取点云数据
point_cloud = k4a.get_point_cloud()

# 将点云转换为Open3D点云
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
pcd.colors = o3d.utility.Vector3dVector(point_cloud[:, 3:])

# 将点云保存到文件中
o3d.io.write_point_cloud("point_cloud.ply", pcd)

# 停止相机
k4a.stop()