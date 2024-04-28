import sys
import torch
import matplotlib.pyplot as plt
import numpy as np
from forward_pass import Plan_Network
from data_loader import RolloutDataset
#sys.path.insert(0, "../path_generation")
#import metropolis

# imu is a 1-D array of length 3(position) + 9(attitude as rot mat) + 3(velocity) + 3(goal direction) + 1 (rollout id) = 19
def Collission_Loss(point_cloud_trees, imu, y_pred, different_pct=False): 
    # imu shape : [batch_size, len(imu_obj) = 18/19]
    # y_pred shape: [batch_size, 3*traj_waypoints_number + 1 , modes]
    # if different_pct=False, then point_cloud_tree is a single tree
    # else, point_cloud_tree is a list of trees, of len batch_size 

    batch_size = imu.size(dim=0)
    modes = y_pred.size(dim=2)
    pct = None
    if not different_pct:
        pct = point_cloud_trees
    
    mse = torch.nn.MSELoss()
    trajectory_costs = []
    pred_costs = y_pred[:, 0, :].reshape(batch_size, modes) # shape = [batch_size, 1, modes] ---> [batch_size, modes]
    for a in range(batch_size):
        init_imu = imu[a]
        trajectory = y_pred[a]
        if different_pct:
            pct = point_cloud_trees[a]
        trajectory_costs.append(traj_calc_cost(pct, init_imu, trajectory)) # .detach()
    tc_cost = torch.stack(trajectory_costs) # shape: [batch_size, modes]
    tc_loss = 2*mse(pred_costs, tc_cost)
    return tc_loss

def traj_calc_cost(pct, init_imu, trajectory):
    # pct is a single point cloud tree
    # trajectory shape: [3*10 + 1 , modes = 3]
    # init_imu shape: [len(imu_obj) = 18/19]
    # output tensor size: (modes)  
    radius_quad = 0.2
    threshold = 0.41
    wf_traj = Transform_to_world_frame(trajectory, init_imu)
    modes = trajectory.size(dim=1)
    cost = []
    for a in range(modes):
        mode_cost = 0
        traj = wf_traj[1:,a]
        points = traj.reshape(-1, 3) # The kth trajectory point is retrieved by points[k] which will be a 1d tensor [x, y, z]
        for b in range(points.size(dim=0)):
            test_point = points[b].detach().numpy() # Fix later
            [_, indices, sqr_dist] = pct.search_radius_vector_3d(test_point, threshold)
            if indices:
                d_c = np.sqrt(np.min(sqr_dist))
            else:
                d_c = 3 * radius_quad
            
            if d_c > 2 * radius_quad:
                C_threshold = 0
            else:
                C_threshold = -(d_c**2)/(radius_quad**2) + 4
            mode_cost += C_threshold
        cost.append(mode_cost/points.size(dim=0))
    cost = torch.FloatTensor(cost) # shape: [modes]
    return cost

def Transform_to_world_frame(trajectory, init_imu):
    # trajectory shape: [3*10 + 1 , modes = 3]
    # init_imu shape: [len(imu_obj) = 18/19]
    modes = trajectory.size(dim=1)
    pos = init_imu[:3].reshape(3, 1)
    R = init_imu[3:12].reshape(3, 3)
    world_traj = torch.zeros(trajectory.size())
    world_traj[0] = trajectory[0] # copying over the predicted cost
    traj = trajectory[1:].reshape(modes, 3, -1)
    w_traj = torch.matmul(R, traj) + pos
    w_traj = w_traj.reshape(-1, modes)
    world_traj[1:] = w_traj
    return world_traj

def Batch_Trajectory_Loss(batch_expert_traj, batch_pred_traj):
    # batch_*_traj shape : [batch, 3*10 + 1, modes = 3]
    batch_size, _, _ = batch_expert_traj.shape
    loss_list = []
    for i in range(batch_size):
        loss = Trajectory_Loss(batch_expert_traj[i], batch_pred_traj[i])
        loss_list.append(loss)
    loss = torch.stack(loss_list) # loss shape: [batch]
    return torch.mean(loss)

def Trajectory_Loss(expert_traj, pred_traj):
    # trajectory shape: [3*10 + 1 , modes = 3]
    modes = expert_traj.size(dim=1)
    epsilon = 0.05
    traj_loss = 0
    sqr_norm = np.zeros(shape=(modes,modes))
    for a in range(modes):
        single_pred_traj = pred_traj[1:, a].reshape(-1, 3)
        for b in range(modes):
            single_expert_traj = expert_traj[1:, b].reshape(-1, 3)
            sqr_norm_diff = torch.norm(single_expert_traj - single_pred_traj)**2
            sqr_norm[a, b] = sqr_norm_diff
            #   [(expert0, pred0), (expert1, pred0), (expert2, pred0)]   
            #   [(expert0, pred1), (expert1, pred1), (expert2, pred1)]
            #   [(expert0, pred2), (expert1, pred2), (expert2, pred2)]
    for a in range(modes):
        single_pred_traj = pred_traj[1:, a].reshape(-1, 3)
        for b in range(modes):
            single_expert_traj = expert_traj[1:, b].reshape(-1, 3)
            sqr_norm_diff = torch.norm(single_expert_traj - single_pred_traj)**2
            if sqr_norm_diff == np.min(sqr_norm[a, :]):
                traj_loss += (1 - epsilon) * sqr_norm_diff
            else:
                traj_loss += (epsilon/(modes - 1)) * sqr_norm_diff
    return traj_loss

def total_loss(L_collision,L_trajectory):
    lambda_collision = 0.1
    lambda_trajectory = 10
    loss = lambda_collision*L_collision + lambda_trajectory*L_trajectory
    return loss


def train_loop():
    # Get data
    rollout_folder = "rollout_21-02-06_15-12-42"
    dt = RolloutDataset(rollout_folder)
    dataset = dt.__getdataset__(0)
    input_image = dataset["depths"][0].unsqueeze(0)
    input_imu = dataset["imu"][0].unsqueeze(0)
    expert_traj = dataset["trajectories"][0].unsqueeze(0)
    exp = torch.ones(expert_traj.size())
    exp[:, 0, :] = expert_traj[:, -1, :]
    exp[:, 1:, :] = expert_traj[:,:-1,:]
    input = [input_image, input_imu]
    print("input is loaded")

    # Create network
    net = Plan_Network()
    net.train()

    # Define optimizer
    optimizer = torch.optim.SGD(net.parameters(), lr=0.0001)
    loss_arr = []
    for i in range(len(dataset["depths"])):
        # Get data
        input_image = dataset["depths"][i % len(dataset["depths"])].unsqueeze(0)
        input_imu = dataset["imu"][i % len(dataset["depths"])].unsqueeze(0)
        expert_traj = dataset["trajectories"][i % len(dataset["depths"])].unsqueeze(0)
        point_cloud_trees = dataset["kdtree"]
        exp = torch.ones(expert_traj.size())
        exp[:, 0, :] = expert_traj[:, -1, :]
        exp[:, 1:, :] = expert_traj[:,:-1,:] # weights should be on first row not the last row
        input = [input_image, input_imu]

        # Forward pass
        output = net.forward(input)

        # Calculate loss
        collision_loss = Collission_Loss(point_cloud_trees, input_imu, output, different_pct=False)
        traj_loss = Trajectory_Loss(exp[0], output[0])
        loss = total_loss(collision_loss,traj_loss)
        print(loss)
        loss_arr.append(loss.item())

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        
        # Update weights
        optimizer.step()
    plt.plot(loss_arr, label='Training Loss', color='blue')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training Loss Over Epochs')
    plt.legend()
    plt.show()


train_loop()