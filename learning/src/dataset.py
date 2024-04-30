import os

import cv2
import torch
import numpy as np
import open3d as o3d

from PIL import Image
from torchvision import transforms
from torch.utils.data import Dataset
from scipy.spatial.transform import Rotation as R

from tqdm import tqdm

from util import walk, rollout_samples
from visualize import visualize_rollout, visualize_depth

def load_trajectory(path, top_traj=3, out_seq_len=10):
    traj = np.load(path)
    k = np.minimum(top_traj, traj.shape[0])
    label_length = 3 * out_seq_len

    traj_set = np.zeros((top_traj, label_length))
    traj_set[:k] = traj[:k,:-1]

    if k < top_traj:
        traj_set[k:] = np.repeat(np.expand_dims(traj_set[0], 0), [top_traj - k], axis=0)

    return torch.tensor(np.array(traj_set, dtype=np.float32), dtype=torch.float32)


class RolloutDataset(Dataset):

    def __init__(self, data_dir, config, transform):

        self.transform = transform
        self.config = config

        save = lambda path : "rollout" in os.path.basename(path) and os.path.isdir(path)
        self.rollouts = walk(data_dir, save, search=True)

        if self.config["max_rollouts"] != -1:
            self.rollouts = self.rollouts[:self.config["max_rollouts"]]

        self.samples = []
        self.pc_trees = []
        self.sample_idxs = []

        # for visuals
        self.point_clouds = []


        for i, rollout in enumerate(tqdm(self.rollouts, desc="Rollouts")):
            samples_subset, pcd, ref_traj = rollout_samples(rollout, i, out_seq_len=self.config["out_seq_len"], reset=self.config["reset"], future_time=self.config["future_time"])

            pc = o3d.io.read_point_cloud(pcd)
            pc_tree = o3d.geometry.KDTreeFlann(pc)

            self.point_clouds.append(pc)
            self.pc_trees.append(pc_tree)
            self.sample_idxs.append(list(range(len(self.samples), len(self.samples) + len(samples_subset))))
            self.samples += samples_subset

    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        gt_depth = self.samples[idx][0]
        gt_depth = cv2.imread(gt_depth, cv2.IMREAD_ANYDEPTH)
        gt_depth = np.minimum(gt_depth, 20000) / 80
        gt_depth = Image.fromarray(gt_depth.astype('uint8'))
        gt_depth = gt_depth.convert('RGB')
        gt_depth = self.transform(gt_depth)

        gt_traj = self.samples[idx][1]
        gt_traj = load_trajectory(gt_traj, top_traj=self.config["top_traj"], out_seq_len=self.config["out_seq_len"])

        imu_obs = torch.tensor(self.samples[idx][2], dtype=torch.float32)

        sample = { 'gt_depth': gt_depth, 'gt_traj': gt_traj, 'imu_obs' : imu_obs, 'traj_idx' : self.samples[idx][3], 'rollout_id' : self.samples[idx][4] } 

        return sample


def build_dataset(dir, config):
    return RolloutDataset(dir, 
        config, 
        transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.3964, 0.3964, 0.3964], std=[0.2910, 0.2910, 0.2910]), # Precalculated to the mean and std of the dataset
            ])
    )

if __name__ == "__main__":

    config = {
        "out_seq_len" : 10,
        "reset" : True,
        "future_time" : 5.0,
        "top_traj" : 3,
        "max_rollouts" : 2
    }
    
    dataset = RolloutDataset("../data/dataset/train_all_7ms/train", 
        config, 
        transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.3964, 0.3964, 0.3964], std=[0.2910, 0.2910, 0.2910]), # Precalculated to the mean and std of the dataset
            ])
    )

    print(dataset.sample_idxs)
    visualize_rollout(dataset, dataset.sample_idxs[1][::10])