import torch
import numpy as np
import open3d as o3d
import torch.nn as nn

from torchvision import models
from torch.utils.data import Dataset, DataLoader

class TrajectoryNetwork(nn.Module):

    def __init__(self, config):
        super(TrajectoryNetwork, self).__init__()

        M, output_size = config["modes"], config["output_size"]

        self.backbone = models.mobilenet_v3_large(pretrained=True, weights=models.MobileNet_V3_Large_Weights.IMAGENET1K_V1)
        self.backbone = self.backbone.features

        self.resize_op1 = nn.Conv1d(960, 128, kernel_size=1, stride=1, padding="valid", dilation=1, bias=True)

        self.img_mergenet = nn.Sequential(
            nn.Linear(6272, 128),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Linear(128, 64),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Linear(64, 64),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Linear(64, 32),
            nn.LeakyReLU(negative_slope=1e-2)
        )

        self.resize_op2 = nn.Conv1d(1, M, kernel_size=3, stride=1, padding="valid", dilation=1, bias=True)

        self.state_mergenet = nn.Sequential(
            nn.Linear(21, 64),
            nn.LeakyReLU(negative_slope=.5),
            nn.Linear(64, 32),
            nn.LeakyReLU(negative_slope=.5),
            nn.Linear(32, 32),
            nn.LeakyReLU(negative_slope=.5),
            nn.Linear(32, 32),
        )

        self.resize_op3 = nn.Conv1d(1, M, kernel_size=3, stride=1, padding="valid", dilation=1, bias=True)

        self.plan_policy = nn.Sequential(
            nn.Conv1d(60, 64, kernel_size=1, stride=1, padding="valid", dilation=1, bias=True),
            nn.LeakyReLU(negative_slope=.5),
            nn.Conv1d(64, 128, kernel_size=1, stride=1, padding="valid", dilation=1, bias=True),
            nn.LeakyReLU(negative_slope=.5),
            nn.Conv1d(128, 128, kernel_size=1, stride=1, padding="valid", dilation=1, bias=True),
            nn.LeakyReLU(negative_slope=.5),
            nn.Conv1d(128, output_size, kernel_size=1, stride=1, padding="same", dilation=1, bias=True),
        )


    def forward(self, img_batch, imu_batch):
        
        # print(img_batch.size())

        img_ft = self.backbone(img_batch)
        B, C, W, H = img_ft.size()
        
        img_ft = img_ft.view(B, C, W * H)
        img_ft = self.resize_op1(img_ft)
        img_ft = img_ft.view(B, 128 * W * H)

        img_ft = self.img_mergenet(img_ft)
        img_ft = img_ft.unsqueeze(1)

        img_ft = self.resize_op2(img_ft)

        imu_ft = self.state_mergenet(imu_batch)
        imu_ft = imu_ft.unsqueeze(1)

        imu_ft = self.resize_op3(imu_ft)

        cat_ft = torch.cat((img_ft, imu_ft), dim=2)
        cat_ft = cat_ft.permute(0, 2, 1).contiguous()

        out = self.plan_policy(cat_ft)
        out = out.permute(0, 2, 1).contiguous()

        # print("out", out.size())

        return out


class TrajectoryCostLoss:

    def __init__(self, pcds):
        self.mse_loss = nn.MSELoss()
        self.pcds = pcds
    
    def __call__(self, imu_obs, ids, y_pred):
        B, M, O = y_pred.size()

        alphas = y_pred[:, :, 0]
        gt_alphas = None

        with torch.no_grad():
            traj_costs = []

            for k in range(B):
                traj_costs.append(self._compute_trajs_costs(imu_obs[k], ids[k], y_pred[k, :, 1:]))

            gt_alphas = torch.stack(traj_costs)
        
        # print(alphas.size())
        # print(gt_alphas)

        loss = 2 * self.mse_loss(gt_alphas, alphas)
        return loss

    def _compute_trajs_costs(self, imu_obs, id, y_pred):
        M, O = y_pred.size()
        costs = []

        for k in range(M):
            cost = self._compute_single_traj_cost(imu_obs, id, y_pred[k])
            costs.append(cost)
        
        return torch.tensor(costs, dtype=y_pred.dtype, device=y_pred.device)
        
    def _compute_single_traj_cost(self, imu_obs, id, pred_traj):
        pcd_tree = self.pcds[id]

        np_traj = pred_traj.numpy()

        imu_obs = imu_obs.numpy()
        rot_mat = imu_obs[3:12].reshape(3, 3)
        pos = imu_obs[0:3].reshape(3, 1)

        traj_len = np_traj.shape[0] // 3
        collision_threshold = 0.8
        quadrotor_size = 0.3

        np_traj = np.reshape(np_traj, ((-1, traj_len)))
        np_traj = rot_mat @ np_traj + pos

        cost = 0.0

        for j in range(traj_len):
            [_, __, dists_squared] = pcd_tree.search_radius_vector_3d(np_traj[:, j], collision_threshold)
            if len(dists_squared) > 0:
                dist = np.sqrt(np.min(dists_squared))
                if dist < quadrotor_size:
                    # collision
                    # parabolic cost with vertex
                    cost += -2. / (quadrotor_size**2) * dist ** 2 + 4.
                else:
                    # linear decrease with collision_threshold
                    cost += 2 *(quadrotor_size - dist) / (collision_threshold - quadrotor_size) + 2
        
        return cost / traj_len # average cost

class MixtureSpaceLoss:

    def __init__(self):
        self.eps = 0.05

    def __call__(self, y_true, y_pred):
        B, M, O = y_pred.size()

        mode_losses = torch.zeros((B, M, M), dtype=y_pred.dtype, device=y_pred.device)
        cost_factor = torch.full((B, M, M), self.eps / (M - 1), dtype=y_pred.dtype, device=y_pred.device)

        # print(y_true)
        # print(y_pred)

        for n in range(M):
            pred = y_pred[:, n, 1:].reshape(-1, O - 1)
            for e in range(M):
                # print("Fuck", y_true[:, e])
                # print("Damn", pred)
                mode_losses[:, n, e] = torch.sum((y_true[:, e] - pred) ** 2, dim=-1)
        
        idx = torch.argmin(mode_losses, dim=-1).to(y_pred.device)
        
        mask = torch.zeros_like(cost_factor, dtype=torch.bool, device=y_pred.device)
        mask.scatter_(-1, idx.unsqueeze(-1), True)
        
        cost_factor[mask] = 1 - self.eps

        loss = (mode_losses * cost_factor).sum()

        return loss


from dataset import build_dataset
from visualize import visualize_rollout, visualize_depth

if __name__ == "__main__":

    def batch2sample(batch, idx):
        sample = {}
        for k, v in batch.items():
            sample[k] = v[idx]
        return sample

    config = {
        "out_seq_len" : 10,
        "reset" : True,
        "future_time" : 5.0,
        "top_traj" : 3,
        "max_rollouts" : 2,
        "modes" : 3,
        "output_size" : 31
    }

    dataset = build_dataset("../data/dataset/train_all_7ms/train", config)

    dataloader = DataLoader(dataset, batch_size=4, shuffle=True, num_workers=0)

    batch = next(iter(dataloader))

    # print(batch['gt_depth'].size())
    # print(batch['gt_traj'].size())
    # print(batch['imu_obs'].size())
    # print(batch['rollout_id'].size())

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    for k, v in batch.items():
        v.to(device)
    
    batch['gt_depth'].to(torch.float32)
    batch['imu_obs'].to(torch.float32)
    batch['gt_traj'].to(torch.float32)


    # visualize_depth(batch['gt_depth'][0].numpy()[0, :, :])
    
    network = TrajectoryNetwork(config)
    network.load_state_dict(torch.load("../data/models/model_20240429_133800_60"))
    network.to(device)

    trajectory_loss = TrajectoryCostLoss(dataset.pc_trees)
    mixture_space_loss = MixtureSpaceLoss()

    y_pred = network(batch['gt_depth'], batch['imu_obs'])
    traj_loss = trajectory_loss(batch['imu_obs'], batch['rollout_id'], y_pred)
    space_loss = mixture_space_loss(batch['gt_traj'], y_pred)

    with torch.no_grad():
        visualize_rollout(dataset, batch2sample(batch, 0), y_pred[0])

    # print(y_pred)
    print(traj_loss)
    print(space_loss)


    # backbone = models.mobilenet_v3_large(pretrained=True, weights=models.MobileNet_V3_Large_Weights.IMAGENET1K_V1)
    # backbone = backbone.features

    # train_nodes, eval_nodes = get_graph_node_names(backbone)

    # print(train_nodes)
    # print(eval_nodes)

    # summary(backbone, (4, 3, 224, 224))