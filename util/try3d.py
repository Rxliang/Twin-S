import open3d as o3d
import numpy as np
print("Load a ply point cloud, print it, and render it")

pcd = o3d.io.read_point_cloud("cropped_exp_4.ply")
# xyz_load = np.asarray(pcd.points)
# print(xyz_load.shape)
# vis = o3d.visualization.Visualizer()
# o3d.visualization.draw_geometries([pcd])
# vis.create_window()
# vis.add_geometry(pcd)
# vis.run()


print("Demo for manual geometry cropping")
print(
    "1) Press 'Y' twice to align geometry with negative direction of y-axis"
)
print("2) Press 'K' to lock screen and to switch to selection mode")
print("3) Drag for rectangle selection,")
print("   or use ctrl + left click for polygon selection")
print("4) Press 'C' to get a selected geometry")
print("5) Press 'S' to save the selected geometry")
print("6) Press 'F' to switch to freeview mode")
# pcd_data = o3d.data.DemoICPPointClouds()
# pcd = o3d.io.read_point_cloud(pcd_data.paths[0])
o3d.visualization.draw_geometries_with_editing([pcd])

