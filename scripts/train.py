import sys
import torch
sys.path.insert(0, "../path_generation")
import metropolis

# imu is a 1-D array of length 3(position) + 9(attitude as rot mat) + 3(velocity) + 3(goal direction) + 1 (rollout id) = 19
def Trajectory_Cost_Loss(point_cloud_trees, imu, y_pred, different_pct=False): 
    # imu shape : [batch_size, len(imu_obj) = 18/19]
    # y_pred shape: [batch_size, 3*traj_waypoints_number + 1 , modes]
    # if different_pct=False, then point_cloud_tree is a single tree
    # else, point_cloud_tree is a list of trees, of len batch_size 

    batch_size = imu.size(dim=0)
    modes = y_pred.size(dim=3)
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
        trajectory_costs.append(traj_calc_cost(pct, init_imu, trajectory))
    tc_cost = torch.stack(trajectory_costs) # shape: [batch_size, modes]
    tc_loss = 2*mse(pred_costs, tc_cost)
    return tc_loss

def traj_calc_cost(pct, init_imu, trajectory):
    # pct is a single point cloud tree
    # trajectory shape: [3*10 + 1 , modes = 3]
    # init_imu shape: [len(imu_obj) = 18/19]
    # output tensor size: (modes)  
    wf_traj = Transform_to_world_frame(trajectory, init_imu)
    modes = trajectory.size(dim=1)
    cost = []
    for a in range(modes):
        traj = wf_traj[a]
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
    w_traj = torch.tensordot(R, traj, dims=3) + pos
    w_traj = w_traj.reshape(1, -1, modes)
    world_traj[1:] = w_traj
    return world_traj
