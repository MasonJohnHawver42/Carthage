import argparse
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import random
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline
import open3d as o3d
import pandas as pd
import csv
import os
import shutil


def get_nominal_path(nom_path):
    given_path = pd.read_csv(nom_path)
    return given_path

def sample_test_path(path):
    first_time_step = path.iloc[0]
    prev_time = first_time_step['time_from_start']
    time_step = prev_time
    sampled_path = path[path['time_from_start']==prev_time]
    count = 1
    for time in path['time_from_start']:
        if time >= (time_step+0.1):
            lower_percent = (time - (0.1+time_step))/(time-prev_time)
            upper_percent = (0.1+time_step - prev_time)/(time-prev_time)
            lower_labels = path[path['time_from_start']==prev_time]
            upper_labels = path[path['time_from_start']==time]
            time_step+=0.1
            row_upper = upper_labels.iloc[0]
            row_lower = lower_labels.iloc[0]
            new_labels = row_lower*lower_percent+row_upper*upper_percent

            # Create the DataFrame with the provided values
            new_row = pd.DataFrame({
                'time_from_start': new_labels['time_from_start'],  # 'time_from_start' already rounded
                'pos_x': new_labels['pos_x'],
                'pos_y': new_labels['pos_y'],
                'pos_z': new_labels['pos_z'],
                'vel_x': new_labels['vel_x'],
                'vel_y': new_labels['vel_y'],
                'vel_z': new_labels['vel_z'],
                'acc_x': new_labels['acc_x'],
                'acc_y': new_labels['acc_y'],
                'acc_z': new_labels['acc_z'],
                'q_w': new_labels['q_w'],
                'q_x': new_labels['q_x'],
                'q_y': new_labels['q_y'],
                'q_z': new_labels['q_z'],
                'omega_x': new_labels['omega_x'],
                'omega_y': new_labels['omega_y'],
                'omega_z': new_labels['omega_z'],
                'reference_progress': new_labels['reference_progress'],
                'pitch_angle': new_labels['pitch_angle']
            },index = [count])
            count += 1
            # Concatenate the new row with the original DataFrame
            sampled_path = pd.concat([sampled_path, new_row], ignore_index=True)
        prev_time = time
    return sampled_path


def get_environment(pointcloud_path):
    # Define the file path
    pointcloud_data = o3d.io.read_point_cloud(pointcloud_path)
    point_cloud_tree = o3d.geometry.KDTreeFlann(pointcloud_data)

    test_point = np.array([11.821883,6.802271,-5.069488])
    test_point = test_point.reshape(3, 1)

    collision_threshold = 1
    [_, indices, distances_squared] = point_cloud_tree.search_radius_vector_3d(test_point,collision_threshold)

    # Check if any points are found within the radius
    if indices:
        # Find the closest point and its distance
        closest_distance = np.min(distances_squared)
        print("Closest Distance:", np.sqrt(closest_distance))
    
    return point_cloud_tree

def create_dataset(odometry,reference_trajectory,environment,img_dir):
    # Create the folder
    rollout_dir = 'Rollout'
    os.makedirs('Rollout',exist_ok=True)
    os.makedirs('Rollout/img',exist_ok=True)
    os.makedirs('Rollout/trajectories',exist_ok=True)

    # Copy files to the respective folders
    shutil.copy(odometry, os.path.join(rollout_dir,'odometry.csv'))
    shutil.copy(reference_trajectory, os.path.join(rollout_dir,'reference_trajectory.csv'))
    shutil.copy(environment, os.path.join(rollout_dir,'pointcloud-unity.ply'))

    # Iterate over files in the source directory
    for filename in os.listdir(img_dir):
        if filename.startswith("gt"):  # Check if filename starts with "gt"
            # Construct full paths for source and destination files
            source_file = os.path.join(img_dir, filename)
            destination_file = os.path.join('Rollout/img', filename)
            
            # Copy the file to the destination directory
            shutil.copy(source_file, destination_file)
    

def metropolis_alg(nom_path,point_cloud_tree):
    # Constants
    num_paths = 5000
    variances = [0.1,0.5,0.9,0.95]
    variance_count = 0
    increase_int = 1600
    forward_int = 11
    max_int = 100

    metro_paths = []
    metro_samples = []
    costs = []

    ref_dists = []
    tot_distance = 0
    for i in range(len(nom_path) - 1):
        distance = np.sqrt((nom_path[i]['x'] -nom_path[i+1]['x'])**2 + 
                     (nom_path[i]['y'] - nom_path[i+1]['y'])**2 + 
                     (nom_path[i]['height'] - nom_path[i+1]['height'])**2)
        tot_distance += distance
        ref_dists.append(tot_distance)
    
    for path_num in range(num_paths):
        print('path number')
        print(path_num)
        start_position = nom_path[0]
        if path_num%increase_int == 0:
            variance = variances[variance_count]
            variance_count += 1

        path = []
        path.append(start_position)
        current_position = start_position
        for max_num in range(max_int):
            # Loop over a maxiumum amount of integers
            for forward_num in range(forward_int-1):
                test_point = sample_from_normal_distribution(current_position,nom_path[forward_num],nom_path[forward_num+1],variance,forward_num)
                if (forward_num+1)%5 == 0 and forward_num != 0:
                    path.append(test_point)
                    current_position = test_point
                else:
                    current_position = test_point

            b_spline_points = []
            for p in path:
                b_spline_point = {'x': p['x'],
                                'y': p['y'],
                                'height': p['height']}
                b_spline_points.append(b_spline_point)

            points_to_b_spline = [(path[0]['x'],path[0]['y'],path[0]['height']),(path[1]['x'],path[1]['y'],path[1]['height']),(path[2]['x'],path[2]['y'],path[2]['height'])]
            bspline_x,bspline_y,bspline_z = quadratic_bspline(points_to_b_spline)
            t_eval = np.linspace(0, 1, 100)  # Evaluate over the range [0, 1] with 100 points
            interpolated_x = bspline_x(t_eval)
            interpolated_y = bspline_y(t_eval)
            interpolated_z = bspline_z(t_eval)
            path_to_test = []
            vector = []
            for x,y,z in zip(interpolated_x,interpolated_y,interpolated_z):
                point = {
                    'x': x,
                    'y': y,
                    'height': z
                    }
                path_to_test.append(point)
                vector.append([x,y,z])

            vector = np.array(vector)
            end_position = path[-1]

            # Calculate Euclidean distance between each point in the vector and the target point
            start_pos = np.array([start_position['x'],start_position['y'],start_position['height']])
            end_pos = [end_position['x'],end_position['y'],end_position['height']]

            start_distances = np.linalg.norm(vector - start_pos, axis=1)
            end_distances = np.linalg.norm(vector - end_pos, axis=1)
            
            # Find the index of the point with the smallest distance
            closest_start_index = np.argmin(start_distances)
            closest_end_index = np.argmin(end_distances)
            path_to_test[closest_start_index] = start_position
            path_to_test = path_to_test[closest_start_index:(closest_end_index+50)]

            # Make the test path to be sampled with 0.1 seconds
            sampled_test_path = []
            dist = 0
            count = 0

            # Add first point
            sampled_test_path.append({'x': path_to_test[0]['x'], 'y': path_to_test[0]['y'], 'height': path_to_test[0]['height']})
            for point_ind in range(len(path_to_test)-1):
                p_current = np.array([path_to_test[point_ind]['x'],path_to_test[point_ind]['y'],path_to_test[point_ind]['height']])
                p_next = np.array([path_to_test[point_ind+1]['x'],path_to_test[point_ind+1]['y'],path_to_test[point_ind+1]['height']])
                dist += np.linalg.norm(p_next-p_current)

                if dist >= ref_dists[count]:
                    count += 1
                    sampled_test_path.append({'x': path_to_test[point_ind+1]['x'], 'y': path_to_test[point_ind+1]['y'], 'height': path_to_test[point_ind+1]['height']})
                    if len(sampled_test_path) == 11:
                        break
            
            # Calculate cost of path
            cost_percentage,cost = calc_cost(point_cloud_tree,nom_path,sampled_test_path)
            random_percent = random.random()
            if cost_percentage > random_percent:
                break
            else: 
                path = []
                path_to_test = []
                path.append(start_position)
                current_position = start_position

            if max_num == max_int-1:
                print('Could not find path within 100 iterations')
                path = 'no_path'
                return "no_path","no_samples","no_cost",False

        if path != 'no_path':
            costs.append(cost)
            metro_paths.append(sampled_test_path)
            metro_samples.append(b_spline_points)

        else:
            print(f"No path found using max iterations for iteration {path_num}")
    if len(costs) == 0:
        print('no_path_found')
        return "no_path","no_samples","no_cost",False
    
    else:
        return metro_paths,metro_samples,costs,True


def quadratic_bspline(points):

    # Extract x, y, z coordinates
    x = [point[0] for point in points]
    y = [point[1] for point in points]
    z = [point[2] for point in points]

    t = np.linspace(0, 1, 6)  # 3 points + degree + 1

    # Quadratic B-spline interpolation
    bspline_x = BSpline(t, x, 2)
    bspline_y = BSpline(t, y, 2)
    bspline_z = BSpline(t, z, 2)

    return bspline_x, bspline_y, bspline_z

def sample_from_normal_distribution(current_position,nom_start,nom_next,variance,index):

    # Extract current position values
    x_nom, y_nom, height_nom = nom_next['x'], nom_next['y'], nom_next['height']
    x_nom_before, y_nom_before, height_nom_before = nom_start['x'], nom_start['y'], nom_start['height']
    x_start, y_start, height_start = current_position['x'], current_position['y'], current_position['height']

    # Check difference in x,y,height
    x_diff = x_nom-x_nom_before
    y_diff = y_nom-y_nom_before
    height_diff = height_nom-height_nom_before

    radius_from_nom_start = np.sqrt(x_diff**2 + y_diff**2 + height_diff**2)
    theta_from_nom_start = np.arccos(height_diff / radius_from_nom_start)
    phi_from_nom_start = np.arctan2(y_diff, x_diff)

    if (index+1)%5 == 0 and index != 0:
        # # Create a normal distribution of theta and phi
        theta_variance = variance
        phi_variance = variance

        # Generate random offsets for theta and phi
        theta_offset = np.random.normal(loc=0, scale=theta_variance)
        phi_offset = np.random.normal(loc=0, scale=phi_variance)

        # New phi and theta coordinates
        new_theta_from_nom_start = theta_from_nom_start + theta_offset
        new_phi_from_nom_start = phi_from_nom_start + phi_offset

        # Write back to xyz coordinates
        new_x_from_nom_start = radius_from_nom_start * np.sin(new_theta_from_nom_start) * np.cos(new_phi_from_nom_start)
        new_y_from_nom_start = radius_from_nom_start * np.sin(new_theta_from_nom_start) * np.sin(new_phi_from_nom_start)
        new_z_from_nom_start = radius_from_nom_start * np.cos(new_theta_from_nom_start)
        

    else:
        # Write back to xyz coordinates
        new_x_from_nom_start = radius_from_nom_start * np.sin(theta_from_nom_start) * np.cos(phi_from_nom_start)
        new_y_from_nom_start = radius_from_nom_start * np.sin(theta_from_nom_start) * np.sin(phi_from_nom_start)
        new_z_from_nom_start = radius_from_nom_start * np.cos(theta_from_nom_start)

    # Create and return the new point
    test_point = {
        'x': x_start + new_x_from_nom_start,
        'y': y_start + new_y_from_nom_start,
        'height': height_start + new_z_from_nom_start,
    }

    return test_point
    

def calc_cost(point_cloud_tree,nominal_trajectory,test_trajectory):
    r_q = 0.2
    lambda_c = 100
    Q = np.eye(3)
    radius_to_check = 0.4
    d_c = np.inf
    cost = 0

    for j in range(len(test_trajectory)):
        test_point = np.array([test_trajectory[j]['x'],test_trajectory[j]['y'],test_trajectory[j]['height']])
        [_,indices,distances_squared] = point_cloud_tree.search_radius_vector_3d(test_point,radius_to_check)
        # Check if any points are found within the radius
        if indices:
            d_c = np.sqrt(np.min(distances_squared))
        else:
            d_c = 3*r_q

        if d_c > 2*r_q:
            C_collision = 0
        else:
            C_collision = -(d_c**2)/(r_q**2)+4

        x_diff = test_trajectory[j]['x'] - nominal_trajectory[j]['x']
        y_diff = test_trajectory[j]['y'] - nominal_trajectory[j]['y']
        z_diff = test_trajectory[j]['height'] - nominal_trajectory[j]['height']

        tau = np.array([x_diff,y_diff,z_diff])
        tau = tau.reshape(3,1)
        cost += (lambda_c*C_collision + tau.T@Q@tau)*0.1

    cost_percentage = np.exp(-cost)

    return float(cost_percentage),float(cost)


def visualize_path(path, samples, nom_path=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract x, y, and height values from the path
    x_values = [point['x'] for point in path]
    y_values = [point['y'] for point in path]
    height_values = [point['height'] for point in path]

    # Plot the path
    ax.plot(x_values, y_values, height_values, label='Path')
    ax.scatter(x_values[0], y_values[0], height_values[0], c='g', marker='o', label='Start')
    ax.scatter(x_values[-1], y_values[-1], height_values[-1], c='r', marker='o', label='End')

    # Mark points in between with blue
    num_points = len(path)
    if num_points > 2:
        for i in range(1, num_points - 1):
            ax.scatter(x_values[i], y_values[i], height_values[i], c='b', marker='o')

    # Plot sample points
    sample_x = [point['x'] for point in samples]
    sample_y = [point['y'] for point in samples]
    sample_z = [point['height'] for point in samples]
    ax.scatter(sample_x, sample_y, sample_z, c='k', marker='o', label='Sample Points')

    # Plot nominal path if provided
    if nom_path:
        nom_x_values = [point['x'] for point in nom_path]
        nom_y_values = [point['y'] for point in nom_path]
        nom_height_values = [point['height'] for point in nom_path]
        ax.plot(nom_x_values, nom_y_values, nom_height_values, c='orange', linestyle='--', label='Nominal Path')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    ax.legend()

    plt.show()


def visualize_paths(paths, nom_path=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i, path in enumerate(paths):
        # Extract x, y, and height values from the path
        x_values = [point['x'] for point in path]
        y_values = [point['y'] for point in path]
        height_values = [point['height'] for point in path]

        # Plot the path
        ax.plot(x_values, y_values, height_values, label='Path' if i == 0 else None)

        # Mark start and end points
        ax.scatter(x_values[0], y_values[0], height_values[0], c='g', marker='o', label='Start' if i == 0 else None)
        ax.scatter(x_values[-1], y_values[-1], height_values[-1], c='r', marker='o', label='End' if i == 0 else None)

        # Mark points in between with blue
        num_points = len(path)
        if num_points > 2:
            for i in range(1, num_points - 1):
                ax.scatter(x_values[i], y_values[i], height_values[i], c='b')

    # Plot nominal path if provided
    if nom_path:
        nom_x_values = [point['x'] for point in nom_path]
        nom_y_values = [point['y'] for point in nom_path]
        nom_height_values = [point['height'] for point in nom_path]
        ax.plot(nom_x_values, nom_y_values, nom_height_values, c='orange', linestyle='--', label='Nominal Path')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')

    ax.set_box_aspect([1, 1, 1])
    ax.legend()

    plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', help='Option to visualize')
    parser.add_argument('--path_to_data', type=str, required=True, help='Mandatory argument file to csv file with paths to data is stored')
    args = parser.parse_args()

    # Create dataset structure
    path_to_data = pd.read_csv(args.path_to_data)
    odometry_path = path_to_data['odometry_path'].iloc[0]
    reference_path = path_to_data['reference_path'].iloc[0]
    pointcloud_path = path_to_data['pointcloud_path'].iloc[0]
    img_dir = path_to_data['img_dir'].iloc[0]
    create_dataset(odometry_path,reference_path,pointcloud_path,img_dir)

    # Get nominal path
    nominal_trajectory = get_nominal_path(odometry_path)

    # Find last point that could traverse one second in the furute
    time_at_last_index = nominal_trajectory.iloc[-1]['time_from_start']
    reversed_nominal_trajectory = nominal_trajectory[::-1]
    desired_timestep = reversed_nominal_trajectory[reversed_nominal_trajectory['time_from_start'] <= time_at_last_index-1].iloc[0]['time_from_start']
    last_valid_index = nominal_trajectory[nominal_trajectory['time_from_start'] == desired_timestep].index.tolist()[0]

    for path_step in range(last_valid_index-1):
        # Sample 1 second to the future with 0.1 seconds interval
        time_at_path_step = nominal_trajectory.iloc[path_step]['time_from_start']
        desired_timestep = nominal_trajectory[nominal_trajectory['time_from_start'] >= time_at_path_step+1].iloc[0]['time_from_start']
        next_path_step = nominal_trajectory[nominal_trajectory['time_from_start'] == desired_timestep].index.tolist()[0]
        nom_traj_1_second = sample_test_path(nominal_trajectory[path_step:next_path_step+2])
        # Get environment
        point_cloud_tree = get_environment(pointcloud_path)
        nom_traj_list = []
        # Iterate over each row in the DataFrame
        for index, row in nom_traj_1_second.iterrows():
            # Create a dictionary for the current timestep
            timestep_data = {
                'x': row['pos_x'],
                'y': row['pos_y'],
                'height': row['pos_z']
            }
            nom_traj_list.append(timestep_data)
            
        metro_paths,metro_samples,costs,valid_path_test = metropolis_alg(nom_traj_list,point_cloud_tree)
        if valid_path_test:
            # Enumerate the costs along with their indices and sort them by cost
            sorted_costs_with_indices = sorted(enumerate(costs), key=lambda x: x[1])

            # Get the indices of the lowest three costs
            lowest_three_indices = [index for index, _ in sorted_costs_with_indices[:3]]

            # Write data to CSV file
            csv_names = [f"{prefix}_{index}" for index in range(11) for prefix in ["pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "acc_x", "acc_y", "acc_z"]]
            csv_names.append("cost")

            # Write only the column names as the first row of the CSV
            filename = f"Rollout/trajectories/traj{path_step}.csv"

            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(csv_names)
            lowest_metro_paths = []
            for lowest_cost_ind in lowest_three_indices:
                lowest_metro_paths.append(metro_paths[lowest_cost_ind])
                s = 0
                data_to_csv = []
                for i in range(len(metro_paths[lowest_cost_ind])):
                    print(s)
                    s +=1
                    data_to_csv.append([metro_paths[lowest_cost_ind][i]['x'], metro_paths[lowest_cost_ind][i]['y'], 
                                    metro_paths[lowest_cost_ind][i]['height'], nominal_trajectory.iloc[path_step+i]["vel_x"], 
                                    nominal_trajectory.iloc[path_step+i]["vel_y"], nominal_trajectory.iloc[path_step+i]["vel_z"], 
                                    nominal_trajectory.iloc[path_step+i]["acc_x"], nominal_trajectory.iloc[path_step+i]["acc_y"], 
                                    nominal_trajectory.iloc[path_step+i]["acc_z"],costs[lowest_cost_ind]])
                    
                # Write data to CSV file
                flattened_csv = [item for sublist in data_to_csv for item in sublist]
                filename = f"Rollout/trajectories/traj{path_step}.csv"
                with open(filename, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows([flattened_csv])
            if args.visualize:
                visualize_path(metro_paths[lowest_three_indices[0]],metro_samples[lowest_three_indices[0]],nom_traj_list)
                visualize_paths(lowest_metro_paths,nom_traj_list)

        else:
            print("No path found")
        
        # visualize_path(metro_paths[0],metro_samples[0],nom_traj_list)
        # visualize_paths(metro_paths,nom_traj_list)