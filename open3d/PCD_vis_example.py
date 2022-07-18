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

from OutlierRemoval import display_inlier #removes outliers


if __name__ == '__main__':
    # folder = "fort-pitt"
    # filename = f"myPointClouds/{folder}/{folder}20.ply"
    filename = "testing/1.ply"
    pcd = o3d.io.read_point_cloud(filename)
    print(pcd)
    print(np.asarray(pcd.points))

    # Voxel Downsamping
    # voxel_down_pcd = pcd.voxel_down_sample(voxel_size=500)
    # cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)

    # # remove outliers
    # display_inlier(voxel_down_pcd, ind)
    o3d.visualization.draw_geometries([pcd])
