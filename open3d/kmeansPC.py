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

from sklearn.preprocessing  import MinMaxScaler

if __name__ == '__main__':
    # Get Data
    folder = "dance"
    filename = f"myPointClouds/{folder}/{folder}0.ply"
    pcd = o3d.io.read_point_cloud(filename)
    # pcd = pcd.uniform_down_sample(50)# Every time 50 One sample at a time 
    # pcd.paint_uniform_color([0.5, 0.5, 0.5])# Specifies that the display is grayed out 
    print(pcd)
    points = np.array(pcd.points)

    # Normalize XYZ
    scaler = MinMaxScaler()
    scaler.fit(points)
    points = scaler.transform(points)
    o3d.utility.Vector3dVector(points)

    #pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # K-MEANS
    result = KMeans(n_clusters=6, max_iter=300,copy_x=True).fit(pcd.colors)
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
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

                                      
    # Voxel Downsamping
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=10.0)
    # Remove outliers
    inlier_cloud = cl.select_by_index(ind)
    outlier_cloud = cl.select_by_index(ind, invert=True)        
    
    #  Point cloud display 
    o3d.visualization.draw_geometries([inlier_cloud], # Point cloud list 
                                      window_name="Kmeans Point cloud clustering ",
                                      point_show_normal=False,
                                      width=800,  #  Window width 
                                      height=600)  #  Window height  