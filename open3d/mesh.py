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

import trimesh

if __name__ == '__main__':
    folder = "dance"
    filename = f"myPointClouds/{folder}/{folder}0.ply"
    pcd = o3d.io.read_point_cloud(filename)
    pcd.estimate_normals()

    # estimate radius for rolling ball
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist   

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd,
            o3d.utility.DoubleVector([radius, radius * 2]))

    # create the triangular mesh with the vertices and faces from open3d
    tri_mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles),
                            vertex_normals=np.asarray(mesh.vertex_normals), 
                            vertex_colors = np.asarray(mesh.vertex_colors))

    trimesh.convex.is_convex(tri_mesh)
    
    o3d.io.write_triangle_mesh("trimesh.ply", mesh)
    pcd2 = o3d.io.read_triangle_mesh("trimesh.ply")
    o3d.visualization.draw_geometries([pcd2])
