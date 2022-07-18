import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys 
import glob
sys.path.append('..')
import open3d_tutorial as o3dtut
o3dtut.intereactive = not "CI" in os.environ 

def downsample(pcd):
    # Voxel Downsamping
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    # cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)

filename = "pointclouds/Large/urban_0.ply"
pcd = o3d.io.read_point_cloud(filename)
print(pcd)
print(np.asarray(pcd.points))
print("Downsample the point cloud with a voxel of 0.05")
downpcd = pcd.voxel_down_sample(voxel_size=0.5)
o3d.visualization.draw_geometries([downpcd])

print("Recompute the normal of the downsampled point cloud")
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([downpcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)