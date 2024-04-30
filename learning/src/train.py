import os
from datetime import datetime

import torch
import torchvision

import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from torch.optim.lr_scheduler import CosineAnnealingWarmRestarts

from tqdm import tqdm

from model import TrajectoryNetwork, TrajectoryCostLoss, MixtureSpaceLoss
from dataset import build_dataset
from visualize import visualize_rollout, visualize_depth

class TrajectoryTrainer:

    def __init__(self, config):
        self.config = config

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.train_dataset = build_dataset(config["train_dir"], config)
        self.val_dataset = build_dataset(config["val_dir"], config)

        self.train_loader = DataLoader(self.train_dataset, batch_size=4, shuffle=True)
        self.val_loader = DataLoader(self.val_dataset, batch_size=4, shuffle=False)

        self.network = TrajectoryNetwork(config)

        if config["load_model"] is not None:
            self.network.load_state_dict(torch.load(config["load_model"]))
        
        self.network.to(self.device)

        self.train_trajectory_loss = TrajectoryCostLoss(self.train_dataset.pc_trees)
        self.val_trajectory_loss = TrajectoryCostLoss(self.train_dataset.pc_trees)
        self.mixture_loss = MixtureSpaceLoss()

        total_steps = len(self.train_loader) * config["epochs"]
        initial_lr = 1e-3
        T_mult = 1
        eta_min = 1e-6
        T_0 = 50000

        # tf.keras.experimental.CosineDecayRestarts(
        #     			1e-3,
        #     			50000,
        #     			1.5,
        #     			0.75,
        #     			0.01)

        self.optimizer = optim.Adam(self.network.parameters(), lr=initial_lr)
        self.scheduler = CosineAnnealingWarmRestarts(self.optimizer, T_0=T_0, T_mult=T_mult, eta_min=eta_min)
    

    def train_step(self, epoch):
        running_loss = 0.0
        running_traj_loss = 0.0
        running_space_loss = 0.0
        iters = len(self.train_loader)

        for i, batch in enumerate(self.train_loader):
            
            for k, v in batch.items():
                v.to(self.device)
            
            batch['gt_depth'].to(torch.float32)
            batch['imu_obs'].to(torch.float32)
            batch['gt_traj'].to(torch.float32)

            self.optimizer.zero_grad()

            y_pred = self.network(batch['gt_depth'], batch['imu_obs'])
            
            traj_loss = self.train_trajectory_loss(batch['imu_obs'], batch['rollout_id'], y_pred)
            space_loss = self.mixture_loss(batch['gt_traj'], y_pred)
            
            loss = 10 * space_loss + 0.1 * traj_loss
            
            loss.backward()
            
            self.optimizer.step()
            self.scheduler.step(epoch + (i / iters))

            running_loss += loss.item()
            running_traj_loss += traj_loss.item()
            running_space_loss += space_loss.item()

        return running_loss / iters, running_traj_loss / iters, running_space_loss / iters

    def val_step(self):
        running_loss = 0.0
        running_traj_loss = 0.0
        running_space_loss = 0.0
        iters = len(self.train_loader)

        for i, batch in enumerate(self.val_loader):
            
            for k, v in batch.items():
                v.to(self.device)

            batch['gt_depth'].to(torch.float32)
            batch['imu_obs'].to(torch.float32)
            batch['gt_traj'].to(torch.float32)

            y_pred = self.network(batch['gt_depth'], batch['imu_obs'])
            
            traj_loss = self.val_trajectory_loss(batch['imu_obs'], batch['rollout_id'], y_pred)
            space_loss = self.mixture_loss(batch['gt_traj'], y_pred)
            
            loss = 10 * space_loss + 0.1 * traj_loss

            running_loss += loss.item()
            running_traj_loss += traj_loss.item()
            running_space_loss += space_loss.item()
        
        return running_loss / iters, running_traj_loss / iters, running_space_loss / iters


    def train(self):
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.writer = SummaryWriter(os.path.join(self.config["log_dir"], 'traj_trainer_{}'.format(timestamp)))

        best_loss = float("inf")

        for epoch in tqdm(range(self.config["epochs"]), "Epochs"):

            self.network.train(True)
            t_avg_loss, t_avg_traj_loss, t_avg_space_loss = self.train_step(epoch)

            self.network.eval()

            with torch.no_grad():
                v_avg_loss, v_avg_traj_loss, v_avg_space_loss = self.val_step()

            self.writer.add_scalars('Training vs. Validation Loss', { 'Training' : t_avg_loss, 'Validation' : v_avg_loss }, epoch + 1)
            self.writer.flush()

            if v_avg_loss < best_loss:
                best_loss = v_avg_loss
                model_path = os.path.join(self.config["model_dir"], 'model_{}_{}'.format(timestamp, epoch))
                torch.save(self.network.state_dict(), model_path)


if __name__ == "__main__":

    config = {
        "out_seq_len" : 10,
        "reset" : True,
        "future_time" : 5.0,
        "top_traj" : 3,
        "max_rollouts" : 2,
        "modes" : 3,
        "output_size" : 31,
        "train_dir" : "../data/dataset/train_all_7ms/train",
        "test_dir" : "../data/dataset/train_all_7ms/test",
        "val_dir" : "../data/dataset/train_all_7ms/val",
        "load_model" : None,
        "epochs" : 100,
        "log_dir" : "../data/log",
        "model_dir" : "../data/models" 
    }

    trainer = TrajectoryTrainer(config)
    trainer.train()