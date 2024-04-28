import open3d as o3d
import numpy as np
from forward_pass import Plan_Network
from scipy.spatial.transform import Rotation
import torch
import os
import pandas as pd

# Load Reference Trajectory
rollout_folder = "..\\..\\rollout_21-02-06_22-10-01"
reference_file = os.path.join(rollout_folder, "reference_trajectory.csv")
reference_data = ref_data = pd.read_csv(reference_file, delimiter=',')
ref_traj = ref_data[['pos_x', 'pos_y', 'pos_z']].values # shape [N, 3]
print("reference trajectory shape: ", ref_traj.shape)

# Define Start and goal location from first and last elements of the reference trajectory
start_location = np.array(ref_traj[0]) # np.array([0, 0, 0])
end_location = np.array(ref_traj[-1]) # np.array([1, 1, 1])
print("Start location: ", start_location)
print("End Location: ", end_location)

# Load Odometry
odom_path = os.path.join(rollout_folder, "odometry.csv")
odometry = pd.read_csv(odom_path, delimiter=',')


# Load expert trajectories
trajectory_dir = os.path.join(rollout_folder, "trajectories")
all_expert_trajectories = []
columns_of_interest = []
traj_indexes = []
for i in range(11):
    columns_of_interest.append(f'pos_x_{i}')
    columns_of_interest.append(f'pos_y_{i}')
    columns_of_interest.append(f'pos_z_{i}')
for dirpath, dirs, files in os.walk(trajectory_dir):
    for file in files:
        if file.startswith("trajectories_bf_") and file.endswith(".csv"):
            # If it does, print the full path to the file
            exp_traj_path = os.path.join(dirpath, file)
            expert_trajectory_data = pd.read_csv(exp_traj_path, delimiter=',')
            expert_trajectory = np.array(expert_trajectory_data[columns_of_interest].values).reshape((-1, 11, 3)) # shape [modes, 11, 3] 
            if (expert_trajectory.shape[0] > 0):
                traj_indexes.append(file.split('.')[0].split('_')[-1])
                all_expert_trajectories.append(expert_trajectory)
print("number of trajectories: ", len(all_expert_trajectories))

# bug fix code
# print(all_expert_trajectories[0])
# bf_data = pd.read_csv(os.path.join(rollout_folder, "trajectories\\trajectories_bf_00000001.csv"))
# bf_traj = np.array(bf_data[columns_of_interest].values).reshape((-1, 11, 3))
# print(bf_traj)
# quarternion = odometry[["q_x", "q_y", "q_z", "q_w"]].values[int(traj_indexes[0])].tolist()
# R = Rotation.from_quat(quarternion).as_matrix().reshape((3, 3))
# print("R shape: ", R.shape)
# print("R: ", R)
# pos = odometry[["pos_x", "pos_y", "pos_z"]].values[int(traj_indexes[0])].reshape((3, 1))
# print("pos shape: ", pos.shape)
# print("pos: ", pos)


# transform body frame to world frame using odometry
def transfrom_trajectories_to_world_frame(trajectories, odometry, traj_indexes):
    num_trajectories = len(trajectories)
    for i in range(num_trajectories):
        trajectory = trajectories[i]
        quarternion = odometry[["q_x", "q_y", "q_z", "q_w"]].values[int(traj_indexes[i])].tolist()
        R = Rotation.from_quat(quarternion).as_matrix().reshape((3, 3))
        pos = odometry[["pos_x", "pos_y", "pos_z"]].values[int(traj_indexes[i])].reshape((3, 1))
        trajectories[i] = transform_trajectory_to_world_frame(trajectory, R, pos)

def transform_trajectory_to_world_frame(trajectory, R, pos):
    # Apply the rotation and translation
    for i in range(trajectory.shape[0]):
        trajectory[i] = np.dot(R, trajectory[i].T).T + pos.T
    return trajectory 

def transform_nn_output_to_world_frame(pred_traj, imu_obj):
    # imu_obj has shape [18]
    pred_traj = pred_traj.detach().numpy() # shape [3*10 + 1, mode]
    imu_obj = imu_obj.numpy()
    R = imu_obj[3:12]
    pos = imu_obj[:3]
    return transform_trajectory_to_world_frame(pred_traj, R, pos)

"""
def transformToWorldFrame(trajectory, start_pos, start_att, ref_frame='bf'):
    if ref_frame == 'bf':
        T_W_S = Pose(start_att, start_pos)
    else:
        assert False, "Unknown reference frame."

    for i in range(trajectory.shape[1]):
        bf_pose = Pose(np.eye(3), trajectory[:, i].reshape((3, 1)))
        wf_pose = T_W_S * bf_pose
        trajectory[:, i] = np.squeeze(wf_pose.t)
    return trajectory
"""

transfrom_trajectories_to_world_frame(all_expert_trajectories, odometry, traj_indexes) 
# bf_traj_towf = transform_trajectory_to_world_frame(bf_traj, R, pos)
# print(bf_traj_towf)

############################################

# Load a point cloud from a .ply file
point_cloud = o3d.io.read_point_cloud(os.path.join(rollout_folder, "pointcloud-unity.ply"))
geometries = []

def create_arrow(location, color=[1, 0, 0], upside_down=False):
    # Create the arrow geometry
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.2, cone_radius=0.6,
        cylinder_height=0.7, cone_height=0.3)
    # Compute the rotation matrix to align the arrow with the z-axis
    if upside_down:
        rotation_matrix = np.eye(3)
        rotation_matrix[2, 2] = -1  # Flip the arrow upside down
    else:
        rotation_matrix = np.eye(3)
    # Translate the arrow to the correct position
    arrow.translate(location- np.array([0, 0, 0.7]))
    # Rotate the arrow to align with the z-axis
    arrow.rotate(rotation_matrix, center=(location[0], location[1], location[2]))
    # Set the color of the arrow
    arrow.paint_uniform_color(color)
    return arrow

def create_sphere(center, radius, color=[0, 0, 0]):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)
    sphere.translate(center)
    return sphere

def create_line(points, color=[0, 0, 0], width=0.01):
    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector(points)
    lines = []
    for i in range(len(points) - 1):
        lines.append([i, i + 1])
    line.lines = o3d.utility.Vector2iVector(lines)
    line.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
    line_line_width = np.array([width for _ in range(len(lines))])
    line_line_width = o3d.utility.DoubleVector(line_line_width)
    #line.line_width = line_line_width
    return line

def create_spheres(points, radius=0.5, color=[0, 0, 0]):
    spheres = []
    for point in points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.compute_vertex_normals()
        sphere.paint_uniform_color(color)
        sphere.translate(point)
        spheres.append(sphere)
    return spheres

def traj_lines_and_spheres_from_points(multi_mode_traj):
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0], [1, 1, 1]]
    modes = multi_mode_traj.shape[0]
    lines = []
    spheres = []
    for i in range(modes):
        lines.append(create_line(multi_mode_traj[i], colors[i]))
        spheres += create_spheres(multi_mode_traj[i], radius=0.05, color=colors[i])
    return lines, spheres

# Specify the center and radius of the sphere
sphere_center = start_location
sphere_radius = 0.2

sph2_center = end_location
# Create and visualize the sphere
start_sphere = create_sphere(sphere_center, sphere_radius)
end_sphere = create_sphere(sph2_center, sphere_radius)

start_arrow = create_arrow(start_location, color=[1, 0, 0], upside_down=True)  # Red arrow
end_arrow = create_arrow(end_location, color=[0, 0, 1], upside_down=True)  # Blue arrow

geometries += [start_sphere, end_sphere, start_arrow, end_arrow]
#o3d.visualization.draw_geometries([point_cloud] + geometries)

all_lines = []
all_spheres = []
for i in range(len(all_expert_trajectories)):
    lines, spheres = traj_lines_and_spheres_from_points(all_expert_trajectories[i])
    all_lines += lines
    all_spheres += spheres
geometries += all_lines
geometries += all_spheres
#o3d.visualization.draw_geometries([point_cloud] + geometries)

# visualize the trajectories for pointcloud that does not have canopy
points = np.asarray(point_cloud.points)

# Filter the points based on the z-axis range
z_min, z_max = -2.0871169567108154, 2.7
filtered_points = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]

# Create a new point cloud with the filtered points
filtered_point_cloud = o3d.geometry.PointCloud()
filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
o3d.visualization.draw_geometries([filtered_point_cloud] + geometries)
o3d.visualization.draw_geometries([point_cloud] + geometries)



"""
points = np.asarray(point_cloud.points)

x_interval = (np.min(points[:, 0]), np.max(points[:, 0]))
y_interval = (np.min(points[:, 1]), np.max(points[:, 1]))
z_interval = (np.min(points[:, 2]), np.max(points[:, 2]))

print("X-axis interval:", x_interval)
print("Y-axis interval:", y_interval)
print("Z-axis interval:", z_interval)

"""