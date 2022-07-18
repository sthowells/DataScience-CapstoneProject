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

vis = o3d.visualization.Visualizer()
vis.create_window()
pointcloud = o3d.geometry.PointCloud()
to_reset = True
vis.add_geometry(pointcloud)


import math
# FUNCTIONS    
def vector_angle(u, v):
    return np.arccos(np.dot(u,v) / (np.linalg.norm(u)* np.linalg.norm(v)))

def get_floor_plane(pcd, dist_threshold=0.02, visualize=False):
    plane_model, inliers = pcd.segment_plane(distance_threshold=dist_threshold,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model    
    return plane_model



for f in files:
    pcd = o3d.io.read_point_cloud(folder + f)

    #pcd = np.asarray(pcd.points).reshape((-1,3))
    
    #pcd.colors = o3d.utility.Vector3dVector(pcd.astype(np.float) / 255.0)

    #pointcloud.points = o3d.utility.Vector3dVector(pcd)

    # Get the plane equation of the floor → ax+by+cz+d = 0
    floor = get_floor_plane(pcd)
    a, b, c, d = floor
    # Translate plane to coordinate center
    pcd.translate((0,-d/c,0))
    # Calculate rotation angle between plane normal & z-axis
    plane_normal = tuple(floor[:3])
    z_axis = (0,0,1)
    rotation_angle = vector_angle(plane_normal, z_axis)
    # Calculate rotation axis
    plane_normal_length = math.sqrt(a**2 + b**2 + c**2)
    u1 = b / plane_normal_length
    u2 = -a / plane_normal_length
    rotation_axis = (u1, u2, 0)
    # Generate axis-angle representation
    optimization_factor = 1.4
    axis_angle = tuple([x * rotation_angle * optimization_factor for x in rotation_axis])
    # Rotate point cloud
    R = pcd.get_rotation_matrix_from_axis_angle(axis_angle)
    pcd.rotate(R, center=(0,0,0))

    pcd = np.asarray(pcd.points).reshape((-1,3))
    pointcloud.points = o3d.utility.Vector3dVector(pcd)
    vis.update_geometry(pointcloud)

    if to_reset:
        vis.reset_view_point(True)
        to_reset = False

    vis.poll_events()
    vis.update_renderer()

