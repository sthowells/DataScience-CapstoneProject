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

if __name__ == "__main__":

    filename = "pointclouds/Small/office_zigzag_0.ply"
    filename2 = "booknew.ply"
    pcd = o3d.io.read_point_cloud(filename2)

    print("Downsample the point cloud with a voxel of 0.05")

    downpcd = pcd.voxel_down_sample(voxel_size=0.02)

    print("Recompute the normal of the downsampled point cloud")
    o3d.geometry.estimate_normals(downpcd,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd])