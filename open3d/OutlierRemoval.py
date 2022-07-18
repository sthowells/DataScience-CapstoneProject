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

# Function to display point cloud without outliers
def display_inlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    o3d.visualization.draw_geometries([inlier_cloud],
                                        zoom=0.3412,
                                        front=[-0.1932, -0.6690, -0.7176],
                                        lookat=[31542.9997, 33501.03609, 54201.9204],
                                        up=[-0.0144, -0.7293, 0.6839],
                                        width=800, 
                                        height=600,
                                        window_name="Inlier Cloud")

# Function to display point cloud with outliers only
def display_outlier_only(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0]) # outlier points in red
    print(len(np.asarray(outlier_cloud.points)))

    o3d.visualization.draw_geometries([outlier_cloud],
                                        zoom=0.3412,
                                        front=[-0.1932, -0.6690, -0.7176],
                                        lookat=[31542.9997, 33501.03609, 54201.9204],
                                        up=[-0.0144, -0.7293, 0.6839],
                                        width=800, 
                                        height=600,
                                        window_name="Outlier Cloud: showing only outliers")
    

# Function to display point cloud with outliers in Red
def display_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0]) # outlier points in red
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                        zoom=0.3412,
                                        front=[-0.1932, -0.6690, -0.7176],
                                        lookat=[31542.9997, 33501.03609, 54201.9204],
                                        up=[-0.0144, -0.7293, 0.6839],
                                        width=800, 
                                        height=600,
                                        window_name="Outlier Cloud: outliers in Red")

if __name__ == '__main__':
    # Get Data
    
    filename = f"newPointClouds/Large/0.2_Depth/StrayKids-1080p-0.2_1.ply"
    
    folder = "Large"
    filename1 = f"fromGoogleDrive/AnalysisPC/output/{folder}/LSMCDD/C1/LSMCDD_0.ply"
    filename2 = "POINTCLOUDS/myPointClouds/dance/dance0.ply"
    pcd = o3d.io.read_point_cloud(filename2)

    # Display point cloud with no outlier detections
    o3d.visualization.draw_geometries([pcd],
                                        zoom=0.3412,
                                        front=[-0.1932, -0.6690, -0.7176],
                                        lookat=[31542.9997, 33501.03609, 54201.9204],
                                        up=[-0.0144, -0.7293, 0.6839],
                                        width=800, 
                                        height=600,
                                        window_name="Raw point cloud input: No outlier detections")

    # Voxel Downsamping
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)

    print("Statistical oulier displayed in red")
    display_outlier(voxel_down_pcd, ind)
    num_points = len(np.asarray(pcd.points))
    print(num_points)
    print("Statistical oulier only display")
    display_outlier_only(voxel_down_pcd, ind)

    print("Statistical oulier removal")
    display_inlier(voxel_down_pcd, ind)