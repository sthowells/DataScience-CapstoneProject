import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys 
import glob
from sklearn.cluster import KMeans
sys.path.append('..')
import open3d_tutorial as o3dtut
o3dtut.intereactive = not "CI" in os.environ 

"""
Combination of plane segmentation followed by K-Means clustering.
First, segment the floor as the inlier points.
Second, cluster the outlier points with K-Means.
"""



if __name__ == '__main__':
    filename = "myPointClouds/pointcloudtorch82.ply"
    folder = "dance"
    filename = f"myPointClouds/{folder}/{folder}0.ply"
    pcd = o3d.io.read_point_cloud(filename)

    plane_model, inliers = pcd.segment_plane(distance_threshold=5000,
                                            ransac_n=10,
                                            num_iterations=50)

    # Equation 
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # Select inlier points, indexed by plane model
    inlier_cloud = pcd.select_by_index(inliers)

    # Set inlier points to Red
    inlier_cloud.paint_uniform_color([1.0, 0, 0])

    # invert = True: refers to opposite of inliers
    outlier_cloud = pcd.select_by_index(inliers, invert=True)



    # pcd = pcd.uniform_down_sample(50)
    # # Every time 50 One sample at a time 
    outlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])# Specifies that the display is grayed out 
    print(outlier_cloud)
    
    points = np.array(outlier_cloud.points)
    result = KMeans(n_clusters=8).fit(points)
    # Each category Center 
    center = result.cluster_centers_
    # labels Returns the category of successful clustering , from 0 Start , Each data represents a category 
    labels = result.labels_
     
    # The maximum value is equivalent to how many categories there are 
    max_label = np.max(labels) + 1 # from 0 Start calculating labels 
    print(max(labels))
    # Generate k Colors of categories ,k Indicates the category of successful clustering 
    colors = np.random.randint(255, size=(max_label, 3))/255.
    colors = colors[labels]
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    #  Point cloud display 
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], # Point cloud list 
                                      window_name="RANSAC plane segmentation with K-Means clustering ",
                                      point_show_normal=False,
                                      width=800,  #  Window width 
                                      height=600)  #  Window height 