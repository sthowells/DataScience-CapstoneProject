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
import time
from time import sleep


folder = "myPointClouds/dance/"
files = os.listdir(folder)

"""
from OutlierRemoval import display_inlier

def readpcd(folder, file):
    pcd = o3d.io.read_point_cloud(folder + file)
    return pcd

def down(pcd):
    # Voxel Downsamping
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
    return cl

for f in files:
    readpcd(folder, f)
    down(f)
    display_inlier(f)
"""

vis = o3d.visualization.Visualizer()
vis.create_window()
pointcloud = o3d.geometry.PointCloud()

to_reset = True
vis.add_geometry(pointcloud)

for f in files:
    pcd = o3d.io.read_point_cloud(folder + f)
    
    pts = np.asarray(pcd.points).reshape((-1,3))
    pointcloud.points = o3d.utility.Vector3dVector(pts)

    col = pcd.colors
    pointcloud.colors = o3d.utility.Vector3dVector(col)
        
    vis.update_geometry(pointcloud)

    if to_reset:
        vis.reset_view_point(True)
        to_reset = False

    vis.poll_events()
    vis.update_renderer()

