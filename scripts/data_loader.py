import os
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial.transform import Rotation as R
from PIL import Image
from torchvision import transforms
import torch

class Input():
    def __init__(self, rollout_dir_name):
        self.dir = rollout_dir_name
        self.ptcs = []
        self.odom = []
        self.get_ptc()
        self.get_odom()
        #self.get_depth()
        #self.get_imu()

    def get_ptc(self):
        ptc = o3d.io.read_point_cloud(os.path.join(self.dir, "pointcloud-unity.ply"))
        self.ptcs.append(ptc)
    
    def get_odom(self):
        odom_file = os.path.join(self.dir, "odometry.csv")
        odom = pd.read_csv(odom_file, delimiter=',')
        quarternion = odom[["q_x", "q_y", "q_z", "q_w"]].values.tolist()
        rot = R.from_quat(quarternion).as_matrix()
        rot = rot.flatten().tolist()
        matrix_entry_fmt = "r_{}"
        for i in range(9):
            odom[matrix_entry_fmt.format(i)] = rot[:, i]


class RolloutDataset(): # parse the rollout_director and return dataset
    def __init__(self, rollout_dir):
        self.root_dir = rollout_dir
        self.traj_filenames = []
        self.depth_filenames = []
        self.imu_objs = []
        self.rollout_dir = rollout_dir
    
    def __getdataset__(self, idx):
        rollout_dir = self.rollout_dir
        
        # Load odometry data
        odometry_path = os.path.join(rollout_dir, "odometry.csv")
        odom = pd.read_csv(odometry_path)
        quarternion = odom[["q_x", "q_y", "q_z", "q_w"]].values.tolist()
        #print(quarternion.shape)
        rot = list(map(lambda x: R.from_quat(x).as_matrix().reshape(9,).tolist(), quarternion))
        rot = np.array(rot)
        matrix_entry_fmt = "r_{}"
        for i in range(9):
            odom[matrix_entry_fmt.format(i)] = rot[:, i]
        features_odom = [
            "pos_x",
            "pos_y",
            "pos_z",
            "r_0",
            "r_1",
            "r_2",
            "r_3",
            "r_4",
            "r_5",
            "r_6",
            "r_7",
            "r_8",
            "vel_x",
            "vel_y",
            "vel_z"]
        odometry = odom[features_odom].values
        R_W_C = odometry[:, 3:12].reshape(-1, 3, 3)
        for i in range(R_W_C.shape[0]):
            v_k_w = odometry[i, 12:15].reshape(3, 1)
            v_k_c = R_W_C[i].T @ v_k_w
            odometry[i, 12:15] = v_k_c.T
        self.odometry = odometry 
        #print(odom)

        # Load reference trajectory [used to determine goal direction]
        ref_file = os.path.join(self.rollout_dir, "reference_trajectory.csv")
        ref_data = pd.read_csv(ref_file, delimiter=',')
        reference_positions = ref_data[['pos_x', 'pos_y', 'pos_z']].values
        drone_pos = odom[["pos_x","pos_y","pos_z"]].values
        reference_progress = [1]
        goal_dir = reference_positions[reference_positions.shape[0] - 1] - drone_pos[0]
        goal_dir = goal_dir / np.linalg.norm(goal_dir)
        reference_direction = [goal_dir]
        for i in range(1, drone_pos.shape[0]):
            quad_position = drone_pos[i]
            current_idx = reference_progress[-1]
            reference_position = reference_positions[current_idx]
            distance = np.linalg.norm(reference_position - quad_position)
            if current_idx + 1 >= reference_positions.shape[0]:
                reference_progress.append(current_idx)
                goal_dir = reference_positions[current_idx] - quad_position
                goal_dir /= np.linalg.norm(goal_dir)
                reference_direction.append(goal_dir)
            else:
                for k in range(current_idx + 1, reference_positions.shape[0]):
                    reference_position = reference_positions[k]
                    next_point_distance = np.linalg.norm(reference_position - quad_position)
                    if next_point_distance > distance:
                        reference_progress.append(k-1)
                        future_idx = reference_positions.shape[0] - 1
                        goal_dir = reference_positions[future_idx] - quad_position
                        goal_dir /= np.linalg.norm(goal_dir)
                        reference_direction.append(goal_dir)
                        break
                    else:
                        distance = next_point_distance
                        if k == (reference_positions.shape[0] - 1):
                            reference_progress.append(current_idx)
                            goal_dir = reference_positions[current_idx] - quad_position
                            goal_dir /= np.linalg.norm(goal_dir)
                            reference_direction.append(goal_dir)

        
        # Load point cloud data
        pointcloud_path = os.path.join(rollout_dir, "pointcloud-unity.ply")
        pointcloud_data = o3d.io.read_point_cloud(pointcloud_path)
        pc_tree = o3d.geometry.KDTreeFlann(pointcloud_data)
        self.pc_tree = pc_tree
        
        # Load image data
        image_dir = os.path.join(rollout_dir, "img")
        image_files = sorted([f for f in os.listdir(image_dir) if (f.endswith(".tif") and f.startswith("gt_depth"))])
        transform = transforms.Compose([
        transforms.Lambda(lambda x: Image.open(x).convert('I')),
        transforms.ToTensor(),
        transforms.Lambda(lambda x: x.unsqueeze(-1))]) # shape: [batch_size, H, W, channel=1]
        numbers_img = [f.split('_')[-1].split('.')[0][-3:] for f in image_files]
        
        
        # Load trajectory data
        trajectory_dir = os.path.join(rollout_dir, "trajectories")
        trajectory_files = sorted([os.path.join(trajectory_dir, f) for f in os.listdir(trajectory_dir) if f.endswith(".npy")])
        numbers_traj = [f.split('_')[-1].split('.')[0][-3:] for f in trajectory_files]
        common_numbers = sorted(set(numbers_img) & set(numbers_traj))
        trajectory_files = [os.path.join(trajectory_dir, 'trajectories_bf_00000{}.npy'.format(num)) for num in common_numbers]
        image_files = [os.path.join(image_dir, 'gt_depth_00000{}.tif'.format(num)) for num in common_numbers]
        depth_data = [transform(f) for f in image_files]
        depth_data = torch.cat(depth_data, dim=0)
        trajectory_data = np.array([np.load(f)[:3].T for f in trajectory_files]) # shape = [batch_size, 3*seq_len + 1, modes]
        num_files = trajectory_data.shape[0]
        trajectory_data = torch.tensor(trajectory_data)
        self.trajectory_data = trajectory_data

        for k in range(num_files):
            R_world = odometry[k][3:12].reshape(3,3)
            goal_dir = np.squeeze(R_world @ reference_direction[k].reshape(3, 1))
            self.imu_objs.append(np.concatenate((odometry[k], goal_dir)))
        self.imu_objs = torch.tensor(np.array(self.imu_objs))
        
        return {
            "odometry": odometry,
            "pointcloud": pointcloud_data,
            "kdtree": pc_tree,
            "depths": torch.Tensor(np.array(depth_data)).to(torch.float32),
            "trajectories": torch.Tensor(trajectory_data).to(torch.float32),
            "imu":torch.Tensor(self.imu_objs).to(torch.float32)
        }
        
        
if __name__ == "__main__":
    rollout_dir = os.path.join("..", "rollout_21-02-06_22-10-01")
    dt = RolloutDataset(rollout_dir)
    dataset = dt.__getdataset__(1)
    print(dataset["odometry"].shape)
    ptc = dataset["pointcloud"]
    print(np.asarray(ptc.points).shape)
    print(dataset["depths"].shape)
    print(dataset["trajectories"].shape)
    print(dataset["imu"].shape)



#i = Input("../rollout_21-02-06_22-10-01")
#print(np.asarray(i.ptc.points).shape)