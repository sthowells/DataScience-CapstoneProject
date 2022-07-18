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

# Function to 
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    outlier_cloud.paint_uniform_color([1, 0, 0]) # outlier points in red
    #inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8]) # makes inlier points gray
    o3d.visualization.draw_geometries([inlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

if __name__ == '__main__':
    # Get Data
    folder = "dance"
    filename = f"myPointClouds/{folder}/{folder}0.ply"
    pcd = o3d.io.read_point_cloud(filename)

    pcd = pcd.normalize_normals()
    print(pcd)

    print(np.asarray(pcd.points))

    # Voxel Downsamping
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=1000, std_ratio=2.0)
    
    inlier_cloud = cl.select_by_index(ind)
    outlier_cloud = cl.select_by_index(ind, invert=True)
    

    points = np.array(inlier_cloud.points)
    result = KMeans(n_clusters=4).fit(points)
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
    inlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    #  Point cloud display 
    o3d.visualization.draw_geometries([inlier_cloud], # Point cloud list 
                                      window_name="Kmeans Point cloud clustering ",
                                      point_show_normal=False,
                                      width=800,  #  Window width 
                                      height=600)  #  Window height 