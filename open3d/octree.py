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

filename1 = "pointclouds/Large/urban_0.ply"
filename = "myPointClouds/book/10.ply"
filename2 = "booknew.ply"
pcd = o3d.io.read_point_cloud(filename2)
# np_pcd1 = np.asarray(pcd1.points)
# filename2 = "pointclouds/Small/office_zigzag_0.ply"
# pcd2 = o3d.io.read_point_cloud(filename2)
# np_pcd2 = np.asarray(pcd2.points)


voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=0.05)
# o3d.visualization.draw_geometries([pcd1])
# N = 2000
# pcd1.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))

# print('octree division')
# octree = o3d.geometry.Octree(max_depth=2)
# octree.create_from_voxel_grid(voxel_grid)
# o3d.visualization.draw_geometries([octree])


def f_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1
            print(
                "{}{}: Internal node at depth {} has {} children and {} points ({})"
                .format('    ' * node_info.depth,
                        node_info.child_index, node_info.depth, n,
                        len(node.indices), node_info.origin))

            # we only want to process nodes / spatial regions with enough points
            early_stop = len(node.indices) < 250
    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            print("{}{}: Leaf node at depth {} has {} points with origin {}".
                  format('    ' * node_info.depth, node_info.child_index,
                         node_info.depth, len(node.indices), node_info.origin))
    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop

N = 2000

octree = o3d.geometry.Octree(max_depth=5)
octree.convert_from_point_cloud(pcd, size_expand=0.01)
octree.traverse(f_traverse)
pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
o3d.visualization.draw_geometries([octree])
