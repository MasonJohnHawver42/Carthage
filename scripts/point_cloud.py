from scene import Scene
import point_cloud_utils as pcu
import numpy as np
import transforms3d as tf
import struct
import open3d as o3d


def transformation_matrix(position, quaternion, scale):
    rotation_matrix = tf.quaternions.quat2mat(np.array(quaternion))
    scaling_matrix = np.diag(scale )
    transformation = np.eye(4)
    transformation[:3, :3] = rotation_matrix @ scaling_matrix
    transformation[:3, 3] = position
    print(scaling_matrix, rotation_matrix @ scaling_matrix)
    print(transformation, scale, quaternion)

    return transformation.T

def compute_bounding_box(points):
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)
    return min_coords, max_coords

def combine_bounding_boxes(bounding_box0, bounding_box1):
    combined_min_coords = np.minimum(bounding_box0[0], bounding_box1[0])
    combined_max_coords = np.maximum(bounding_box0[1], bounding_box1[1])
    return combined_min_coords, combined_max_coords

if __name__ == "__main__":

    scn = Scene()
    scn.read("data/scenes/test.scn")

    models = {}

    for obj in scn.objects:
        if obj["fn"] not in models:
            v, f, n = pcu.load_mesh_vfn("./data/" + ".".join(obj["fn"].split(".")[0 : -1]) + ".obj")
            models[obj["fn"]] = {"f" : f, "v" : v}
    
    xyz = []

    bounding_box = (np.array([np.inf, np.inf, np.inf]), np.array([-np.inf, -np.inf, -np.inf]))  # Infinite bounds

    for obj in scn.objects:
        trans_mat = transformation_matrix(obj["pos"], obj["quat"], obj["scale"])
        v, f0 = models[obj["fn"]]["v"], models[obj["fn"]]["f"] 
        v_trans = np.hstack([v, np.ones((v.shape[0], 1))]) @ trans_mat
        v_trans = v_trans[:, 0 : 3]

        bounding_box_obj = compute_bounding_box(v_trans)
        bounding_box = combine_bounding_boxes(bounding_box, bounding_box_obj)
        print(bounding_box_obj, bounding_box)

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(v_trans)
        mesh.triangles = o3d.utility.Vector3iVector(f0)

        print(mesh.get_surface_area())

        N = int(np.floor(mesh.get_surface_area() * 1000))
        pcd = mesh.sample_points_uniformly(N)
        downpcd = pcd.voxel_down_sample(voxel_size=0.01)

        o3d.visualization.draw_geometries([downpcd])

        xyz.append(np.asarray(downpcd.points))

    with open("data/scenes/test.xyz", 'wb') as f:
        f.write(struct.pack('i', sum([len(pts) for pts in xyz])))
        for pts in xyz:
            for i in range(pts.shape[0]):
                f.write(struct.pack('f', pts[i, 0]))
                f.write(struct.pack('f', pts[i, 1]))
                f.write(struct.pack('f', pts[i, 2]))

        print(sum([len(pts) for pts in xyz]))
    
    with open("data/scenes/test.bb", 'wb') as f:
        for pt in bounding_box:
            f.write(struct.pack('f', pt[0]))
            f.write(struct.pack('f', pt[1]))
            f.write(struct.pack('f', pt[2]))





