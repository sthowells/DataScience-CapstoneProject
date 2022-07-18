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

##### Read in ply file #####
# folder = "fort-pitt"
# filename = f"myPointClouds/{folder}/{folder}20.ply"
# pcd = o3d.io.read_point_cloud(filename)
filename2 = "pointclouds/Large/urban_1.ply"
pcd = o3d.io.read_point_cloud(filename2)

print(pcd)
print(np.asarray(pcd.points))

# col = pcd.colors
# #col = pcd.astype(np.float) / 255.0
# pcd.colors = o3d.utility.Vector3dVector(col)
#pointcloud.colors = o3d.utility.Vector3dVector(pcd.astype(np.float) / 255.0)

##### Preprocess: normalize #####
from sklearn.preprocessing  import MinMaxScaler
scaler = MinMaxScaler()
scaler.fit(pcd.points)
X_norm = scaler.transform(pcd.points)
print("\n\nX_norm")
print(X_norm)



##### Numpy to Open3d #####
# Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(X_norm)
o3d.io.write_point_cloud("sync.ply", pcd2)
# view new point cloud that has been normalized
newpcd = o3d.io.read_point_cloud("sync.ply")
# #o3d.visualization.draw_geometries([inlier_cloud])

##### Remove outliers #####
cl, ind = newpcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2.0)
inlier_cloud = cl.select_by_index(ind)
outlier_cloud = cl.select_by_index(ind, invert=True)


#  Point cloud display 
o3d.visualization.draw_geometries([inlier_cloud]) # Point cloud list 






# downpcd = inlier_cloud.voxel_down_sample(voxel_size=0.02)
# downpcd.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
#o3d.visualization.draw_geometries([downpcd], point_show_normal=True) 


# from sklearn.metrics import silhouette_score
# from sklearn.cluster import KMeans
# sil = []
# kmax = 25

# # dissimilarity would not be defined for a single cluster, thus, minimum number of clusters should be 2
# for k in range(2, kmax+1):
#   kmeans = KMeans(n_clusters = k).fit(X_norm)
#   labels = kmeans.labels_
#   sil.append(silhouette_score(X_norm, labels, metric = 'euclidean', sample_size=10000))

# print("Calculating")
# print(sil)

# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     labels = np.array(
#         inlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

# max_label = labels.max()
# print(f"point cloud has {max_label + 1} clusters")
# colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# inlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([inlier_cloud])

# plane_model, inliers = inlierpre.segment_plane(distance_threshold=0.01,
#                                          ransac_n=3,
#                                          num_iterations=1000)
# [a, b, c, d] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# inlier_cloud = inlierpre.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = inlierpre.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


# K-MEANS
from sklearn.cluster import KMeans
result = KMeans(n_clusters=10, max_iter=300).fit(inlier_cloud.points)
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
o3d.visualization.draw_geometries([inlier_cloud]) # Point cloud list 

