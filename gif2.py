import open3d as o3d
import numpy as np
import imageio

# 读取PLY文件
pcd = o3d.io.read_point_cloud("E:\\0910ss\\0910_nerf_ppp.ply")

# 计算点云的中心
center = pcd.get_center()

# 沿着Z轴翻转点云
# points = np.asarray(pcd.points)
# points[:, 2] = -points[:, 2]
# pcd.points = o3d.utility.Vector3dVector(points)

# 创建可视化对象
vis = o3d.visualization.Visualizer()
vis.create_window()

# 设置相机的参数
view_control = vis.get_view_control()

# 添加点云到可视化对象
vis.add_geometry(pcd)

view_control.rotate(0,-390,False)
# 设置旋转角度和轴
angles = np.linspace(0, 360, 120)  # 从0到360度，30帧，加快旋转速度
rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.pi / 35))
# 创建图像列表
images = []

# 旋转并捕捉每一帧
for angle in angles:
    # 创建旋转矩阵
    # rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.radians(angle)))
    pcd.rotate(rotation_matrix, center=center)

    # 更新视图
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    # 获取当前视图的图像
    image = np.asarray(vis.capture_screen_float_buffer())
    image = (image * 255).astype(np.uint8)  # 转换为uint8
    images.append(image)

# 关闭可视化窗口
vis.destroy_window()

# 保存为GIF，减少每帧的持续时间来加快速度
imageio.mimsave("E:\\0910ss\\rotation_z_axis_nerf1.gif", images,duration=0.05)  # 每帧持续时间为0.1秒