import os
import re

import pandas as pd
import numpy as np

from scipy.spatial.transform import Rotation as R

def batch2sample(batch, idx):
    sample = {}
    for k, v in batch.items():
        sample[k] = v[idx]
    return sample

def walk(dir, save, search=False):
    paths = []
    for file in os.listdir(dir):
        file_path = os.path.join(dir, file)
        
        if save(file_path):
            paths.append(file_path)
        
        if not os.path.isdir(file_path) or not search or file in [".", ".."]:
            continue

        paths += walk(file_path, save, search)
    return paths

def pd_quat_rot(df):
    quat_load = ["q_x", "q_y", "q_z", "q_w"]
    quat_vals = df[quat_load].values.tolist()

    quat_mat = list(map(lambda q: R.from_quat(q).as_matrix().reshape((9,)).tolist(), quat_vals))
    quat_mat = np.array(quat_mat, dtype=np.float32)

    for i in range(9):
        df["r_{}".format(i)] = quat_mat[:, i]
    
    return df

def change_reference_frame(rot_mat, x):
    R_World = rot_mat.reshape((3, 3))
    v_Body = R_World.T @ x.reshape([3,1])
    return np.squeeze(v_Body)

def npy_mh_taj(path, num_states=10, reset=False):

    npy_file = path[:-4] + ".npy"

    if os.path.isfile(npy_file):
        if reset:
            os.remove(npy_file)
        else:
            return npy_file

    x_pos_load = ["pos_x_{}".format(i) for i in range(1, num_states + 1)]
    y_pos_load = ["pos_y_{}".format(i) for i in range(1, num_states + 1)]
    z_pos_load = ["pos_z_{}".format(i) for i in range(1, num_states + 1)]
    
    df = pd.read_csv(path, delimiter=',')
    
    if df.shape[0] == 0:
        return "None"

    rel_cost = df["rel_cost"].values
    x_pos = df[x_pos_load].values
    y_pos = df[y_pos_load].values
    z_pos = df[z_pos_load].values

    full_trajectory = np.column_stack((x_pos, y_pos, z_pos, rel_cost))

    assert full_trajectory.shape[-1] == 3 * num_states + 1
    
    np.save(npy_file, full_trajectory)
    return npy_file

def odom_features(path):
    df = pd.read_csv(path, delimiter=',')
    df = pd_quat_rot(df)

    # print(df.shape[0])

    odom_load = ["pos_x", "pos_y", "pos_z", "r_0", "r_1", "r_2", "r_3", "r_4", "r_5", "r_6", "r_7", "r_8", "vel_x", "vel_y", "vel_z", "omega_x", "omega_y", "omega_z"]

    features = df[odom_load].values

    R_World = features[:,3:12].reshape((-1,3,3))
    
    for k in range(R_World.shape[0]):
        R_World_k = R_World[k]
        R_Body_k = R_World_k.T
        P_World = features[k,12:15].reshape((3,1))
        P_Body = R_Body_k @ P_World
        features[k, 12:15] = P_Body.T
    
    return features

def ref_traj_features(path, odom_features, future_time=5.0):
    ref_df = pd.read_csv(path, delimiter=',')

    reference_positions = ref_df[['pos_x', 'pos_y', 'pos_z']].values
    drone_pos = odom_features[:, 0:3]

    reference_progress, reference_direction = calculate_ref_dir(drone_pos=drone_pos, ref_pos=reference_positions, future_time=future_time)
    goal_dirs = [change_reference_frame(odom_features[k][3:12], reference_direction[k]) for k in range(odom_features.shape[0])]
    
    imu_obs = [np.concatenate((odom_features[k], goal_dirs[k])) for k in range(odom_features.shape[0])]

    return reference_progress, reference_direction, goal_dirs, imu_obs

##REWRITE LATTER
def calculate_ref_dir(drone_pos, ref_pos, future_time=5.0):
        reference_progress = [1]
        goal_dir = ref_pos[np.minimum(int(50*future_time),ref_pos.shape[0]-1)] - drone_pos[0]
        goal_dir = goal_dir / np.linalg.norm(goal_dir)
        reference_direction = [goal_dir]
        for j in range(1,drone_pos.shape[0]):
            quad_position = drone_pos[j]
            current_idx = reference_progress[-1]
            reference_position = ref_pos[current_idx]
            distance = np.linalg.norm(reference_position - quad_position)
            if current_idx + 1 >= ref_pos.shape[0]:
                reference_progress.append(current_idx)
                goal_dir = ref_pos[current_idx] - quad_position
                goal_dir = goal_dir / np.linalg.norm(goal_dir)
                reference_direction.append(goal_dir)
            else:
                for k in range(current_idx + 1, ref_pos.shape[0]):
                    reference_position = ref_pos[k]
                    next_point_distance = np.linalg.norm(reference_position - quad_position)
                    if next_point_distance > distance:
                        reference_progress.append(k-1)
                        future_idx = np.minimum(k-1 + int(50*future_time), ref_pos.shape[0] - 1)
                        goal_dir = ref_pos[future_idx] - quad_position
                        goal_dir = goal_dir / np.linalg.norm(goal_dir)
                        reference_direction.append(goal_dir)
                        break
                    else:
                        distance = next_point_distance
                        if k == ref_pos.shape[0] -1:
                            # distance of all next points is larger than current_Idx
                            reference_progress.append(current_idx)
                            goal_dir = ref_pos[current_idx] - quad_position
                            goal_dir = goal_dir / np.linalg.norm(goal_dir)
                            reference_direction.append(goal_dir)

        assert len(reference_progress) == len(reference_direction)
        assert len(reference_progress) == drone_pos.shape[0]

        return reference_progress, reference_direction

def rollout_samples(rollout, rollout_id, out_seq_len=10, reset=True, future_time=5.0):
    save = lambda path : "gt_depth" in os.path.basename(path) and os.path.splitext(path)[1] == ".tif"
    gt_depth = walk(os.path.join(rollout, "img"), save)
    gt_depth = { int(re.findall(r'\d+', os.path.basename(path))[0]) : path for path in gt_depth }
    # gt_depth = { i : gt_depth[key] for i, key in enumerate(sorted(gt_depth.keys())) }
    
    save = lambda path : "trajectories_bf" in os.path.basename(path) and os.path.splitext(path)[1] == ".csv"
    mh_traj_bf = walk(os.path.join(rollout, "trajectories"), save)
    mh_traj_bf = [ npy_mh_taj(path, num_states=out_seq_len, reset=reset) for path in mh_traj_bf ]
    mh_traj_bf = { int(re.findall(r'\d+', os.path.basename(path))[0]) : path for path in mh_traj_bf if path != "None" }
    
    odometry = os.path.join(rollout, "odometry.csv")
    assert os.path.isfile(odometry)

    odom_data = odom_features(odometry)

    num_files = odom_data.shape[0]
    num_images = len(gt_depth)

    assert num_files == num_images

    ref_traj = os.path.join(rollout, "reference_trajectory.csv")
    assert os.path.isfile(ref_traj)

    reference_progress, reference_direction, goal_dirs, imu_obs = ref_traj_features(ref_traj, odom_data, future_time=future_time)
    
    # print(reference_direction)

    pcd = os.path.join(rollout, "pointcloud-unity.ply")
    assert os.path.isfile(pcd)

    samples = [(gt_depth[k + 1], mh_traj_bf[k], imu_obs[k], k, rollout_id) for k in range(num_files) if k + 1 in gt_depth and k in mh_traj_bf and reference_progress[k] > 50 ]

    return samples, pcd, ref_traj