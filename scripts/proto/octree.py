import open3d as o3d
import numpy as np

print('input')
N = 10 * 100000
path = "./output.stl"#o3d.data.ArmadilloMesh().path
mesh = o3d.io.read_triangle_mesh(path)
print(mesh)
pcd = mesh.sample_points_uniformly(N)
print(pcd)
# fit to unit cube
pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
          center=pcd.get_center())
pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
o3d.visualization.draw_geometries([pcd])

print('octree division')
octree = o3d.geometry.Octree(max_depth=6)
octree.convert_from_point_cloud(pcd, size_expand=0.01)
o3d.visualization.draw_geometries([octree])


# import numpy as np
# import trimesh
# import pymeshfix

# import matplotlib.pyplot as plt

# def as_mesh(scene_or_mesh):
#     """
#     Convert a possible scene to a mesh.

#     If conversion occurs, the returned mesh has only vertex and face data.
#     """
#     if isinstance(scene_or_mesh, trimesh.Scene):
#         if len(scene_or_mesh.geometry) == 0:
#             mesh = None  # empty scene
#         else:
#             # we lose texture information here
#             mesh = trimesh.util.concatenate(
#                 tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
#                     for g in scene_or_mesh.geometry.values()))
#     else:
#         assert(isinstance(scene_or_mesh, trimesh.Trimesh))
#         mesh = scene_or_mesh
#     return mesh

# test_mesh = as_mesh(trimesh.load('../data/models/sponza/sponza.obj'))

# print(test_mesh.bounding_box.primitive.extents)

# scale_factor = max(test_mesh.bounding_box.primitive.extents)
# test_mesh.vertices = test_mesh.vertices / scale_factor

# test_mesh.show()

# print(test_mesh.bounding_box.bounds)
# aabb = test_mesh.bounding_box.bounds



# n_acc = 50
# n = 200

# points_acc = None
# mask_acc = None

# for i in range(n_acc):
#     points = np.random.uniform(aabb[0], aabb[1], size=(n,3))

#     theta = np.random.uniform(0, 2 * np.pi, size=(n))
#     phi = np.random.uniform(0, np.pi, size=(n))

#     directions = np.vstack((np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi))).T
#     tol=1e-5

#     rmi = trimesh.ray.ray_triangle.RayMeshIntersector(test_mesh)
#     mask = np.array(rmi.contains_points(points))

#     if i == 0:
#         points_acc = points
#         mask_acc = mask
#     else:
#         points_acc = np.vstack((points_acc, points))
#         mask_acc = np.hstack((mask_acc, mask))
    
#     print(i)

# filtered_points = points_acc[mask_acc]

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(filtered_points[:, 0], filtered_points[:, 1], filtered_points[:, 2], c='blue', marker='o')
# plt.show()



# # intersections, _ = test_mesh.ray.intersects_location(
# #             ray_origins=[points[0, :] - directions[0, :] * tol],
# #             ray_directions=[directions[0, :]],
# #         )

# # print(len(intersections))
