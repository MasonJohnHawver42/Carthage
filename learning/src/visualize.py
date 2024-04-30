import os

import cv2
import numpy as np
import pandas as pd
import open3d as o3d

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.spatial.transform import Rotation as R

def visualize_rollout(dataset, sample, pred=None):
    rollout_id = sample['rollout_id']

    pc = dataset.point_clouds[rollout_id]
    reference_file = os.path.join(dataset.rollouts[rollout_id], "reference_trajectory.csv")
    traj_data = pd.read_csv(reference_file, delimiter=',')

    odom_file = os.path.join(dataset.rollouts[rollout_id], "odometry.csv")
    odom_data = pd.read_csv(odom_file, delimiter=',')

    end_points = traj_data[['pos_x', 'pos_y', 'pos_z']].values[[0, -1], :]


    pc.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(np.asarray(pc.points)), 3)))
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pc, voxel_size=0.5)

    geom = [voxel_grid]

    def mesh_mat(rot, pos):
        return np.asarray(
            [[rot[0, 0], rot[0, 1], rot[0, 2], pos[0]],
            [rot[1, 0], rot[1, 1], rot[1, 2], pos[1]],
            [rot[2, 0], rot[2, 1], rot[2, 2], pos[2]],
            [0.0, 0.0, 0.0, 1.0]])


    def arrow_mesh(rot, pos, color, scale_s=1, scale_w=1):
        mesh = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.5*scale_s, cone_radius=0.75*scale_s, cylinder_height=(2.5*scale_w),
                                                        cone_height=2.0*scale_s, resolution=20, cylinder_split=4, cone_split=1)

        mesh.transform(mesh_mat(rot, pos))
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color(color)
        return mesh

    rot = R.from_quat([1.0, 0.0, 0.0, 0.0]).as_matrix()
    geom.append(arrow_mesh(rot, end_points[0] + np.array([0, 0, 5]), [0.0, 1.0, 0.0]))
    geom.append(arrow_mesh(rot, end_points[1] + np.array([0, 0, 5]), [1.0, 0.0, 0.0]))

    ref_quat = traj_data[['q_w', 'q_x', 'q_y', 'q_z']].values
    ref_pos = traj_data[['pos_x', 'pos_y', 'pos_z']].values

    for i in range(len(traj_data)):
        rot = R.from_quat([ref_quat[i, 1], ref_quat[i, 2], ref_quat[i, 3], ref_quat[i, 0]]).as_matrix()

        pole_mesh = o3d.geometry.TriangleMesh.create_cylinder(0.1, 0.04)
        pole_mesh.transform(mesh_mat(rot, ref_pos[i, :]))
        pole_mesh.compute_vertex_normals()
        pole_mesh.paint_uniform_color([0.8, 0.0, 0.0])
        geom.append(pole_mesh)

    ref_quat = odom_data[['q_w', 'q_x', 'q_y', 'q_z']].values
    ref_pos = odom_data[['pos_x', 'pos_y', 'pos_z']].values

    for i in range(len(odom_data)):
        rot = R.from_quat([ref_quat[i, 1], ref_quat[i, 2], ref_quat[i, 3], ref_quat[i, 0]]).as_matrix()

        pole_mesh = o3d.geometry.TriangleMesh.create_cylinder(0.1, 0.04)
        pole_mesh.transform(mesh_mat(rot, ref_pos[i, :]))
        pole_mesh.compute_vertex_normals()
        pole_mesh.paint_uniform_color([0.8, 0.0, 0.0])
        geom.append(pole_mesh)


    trajs_data = sample['gt_traj'].numpy()
    imu_obs = sample['imu_obs'].numpy()
    rot_mat = imu_obs[3:12].reshape(3, 3)
    pos = imu_obs[0:3].reshape(3, 1)

    velocity = (rot_mat @ imu_obs[12:15].T).T
    velocity_direction = velocity / np.linalg.norm(velocity)
    z_axis = np.array([0, 0, 1])

    rotation_axis = np.cross(z_axis, velocity_direction)
    rotation_angle = np.arccos(np.dot(z_axis, velocity_direction))
    arrow_rot_mat = R.from_rotvec(rotation_angle * rotation_axis).as_matrix()

    mesh = arrow_mesh(arrow_rot_mat, pos.reshape(3), [1, 1, 0], scale_s=.2, scale_w=np.linalg.norm(velocity) / 8)
    geom.append(mesh)

    def mesh_traj(traj, rot, offset, colors):
        spheres = []
        lines = []

        b = .9
        traj_len = traj.shape[1] // 3

        for k in range(traj.shape[0]):
            x_pos = traj[k, 0:10]
            y_pos = traj[k, 10:20]
            z_pos = traj[k, 20:30]
            xyz_pos = np.vstack([x_pos, y_pos, z_pos])
            xyz_pos = rot @ xyz_pos + offset

            for i, p in enumerate(xyz_pos.T):
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
                sphere.translate(p)
                color = colors[k]
                color[0] *= (i / traj_len) * (1 - b) + b
                color[1] *= (i / traj_len) * (1 - b) + b
                color[2] *= (i / traj_len) * (1 - b) + b
                sphere.paint_uniform_color(color)  # Blue color
                spheres.append(sphere)
            
            for i in range(xyz_pos.shape[1] - 1):
                line = o3d.geometry.LineSet()
                line.points = o3d.utility.Vector3dVector([xyz_pos[:, i], xyz_pos[:, i + 1]])
                line.lines = o3d.utility.Vector2iVector([[0, 1]])
                lines.append(line)
        
        return spheres, lines

    spheres, lines = mesh_traj(trajs_data, rot_mat, pos, [[0, 0, 1], [.5, 0, 1], [0, .5, 1]])
    geom.extend(spheres)
    geom.extend(lines)

    if pred is not None:
        pred = pred.numpy()
        spheres, lines = mesh_traj(pred[:, 1:], rot_mat, pos, [[pred[0, 0], 1, 0], [pred[0, 1], 1, 0], [pred[0, 2], 1, 0]])
        geom.extend(spheres)
        geom.extend(lines)
    
    o3d.visualization.draw_geometries(geom)

def visualize_depth(depth_image):
    dim = (64, 64)
    dwn_depth_image = cv2.resize(depth_image, dim)

    x = np.arange(0, dwn_depth_image.shape[1], 1)
    y = np.arange(0, dwn_depth_image.shape[0], 1)
    x, y = np.meshgrid(x, y)

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    # Plot the depth image
    axes[0].imshow(depth_image, cmap='plasma')
    axes[0].set_title('Depth Image')
    axes[0].set_xlabel('X')
    axes[0].set_ylabel('Y')

    # Plot the 3D surface
    axes[1] = fig.add_subplot(122, projection='3d')
    # surf = axes[1].plot_surface(x, depth_image.shape[0] - y, 255 - depth_image, cmap='viridis')
    axes[1].scatter(x, dwn_depth_image, dwn_depth_image.shape[0] - y, c=dwn_depth_image, cmap='viridis', linewidth=0.5);
    axes[1].set_title('Depth Image 3D Surface')
    axes[1].set_xlabel('X')
    axes[1].set_ylabel('Depth')
    axes[1].set_zlabel('Y')

    axes[1].view_init(elev=0, azim=-90, roll=0) 

    # Add a color bar to the 3D surface plot
    # fig.colorbar(surf, ax=axes[1], shrink=0.5, aspect=5)

    # Show the plot
    plt.show()