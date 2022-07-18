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


"""
 Segments the floor -- little cutoff on feet

 distance_threshold = 5000
 defines the maximum distance a point can have to an estimated plane to be considered an inlier

 ransac = 10
 defines the number of points that are randomly sampled to estimate a plane

 num_iter = 50
 defines how often a random plane is sampled and verified

 Plane equation (settings above): 
 -0.01x + 0.86y + -0.52z + -29352.47 = 0
"""

# Read in ply file as point cloud
folder = "dance"
filename = f"myPointClouds/{folder}/{folder}0.ply"
filename1 = "booknew.ply"
pcd = o3d.io.read_point_cloud(filename1)
plane_model, inliers = pcd.segment_plane(distance_threshold=1000,
                                         ransac_n=10,
                                         num_iterations=50)

# Equation 
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

#pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                                                        # radius=0.1, 
                                                        # max_nn=16), 
                                                        # fast_normal_computation=True)

# Select inlier points, indexed by plane model
inlier_cloud = pcd.select_by_index(inliers)

# Set inlier points to Red
inlier_cloud.paint_uniform_color([1.0, 0, 0])

# invert = True: refers to opposite of inliers
outlier_cloud = pcd.select_by_index(inliers, invert=True)

# Set outlier points to different color (example, Blue) if needed
#outlier_cloud.paint_uniform_color([0.0, 0.0, 1.0])

# Draw geometries and set view point
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=0.8,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])

