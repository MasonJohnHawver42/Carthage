import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import random
import struct
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline
import open3d as o3d

def get_nominal_path(velocity):
    # Open the text file for reading
    with open('path_generation/path.txt', 'r') as file:
        # Read the contents of the file
        data = file.read()

    # Split the data into lines and remove empty lines
    lines = data.strip().split('\n')

    # Predefine the coord list
    x_values = []
    y_values = []
    height_values = []
    for line in lines:
        line_split = line.split()
        x_values.append(float(line_split[0]))
        y_values.append(float(line_split[1]))
        height_values.append(float(line_split[2]))

    path = []
    for i in range(len(lines) - 1):
        p1 = {'x': x_values[i], 'y': y_values[i], 'height': height_values[i]}
        p2 = {'x': x_values[i + 1], 'y': y_values[i + 1], 'height': height_values[i + 1]}
        
        # Interpolate 10 points between p1 and p2
        interpolated_points = interpolate_points(p1, p2, 1000)
        
        # Add the points to the path
        path.extend(interpolated_points)    

    # Add the last point
    path.append({'x': x_values[-1], 'y': y_values[-1], 'height': height_values[-1]})

    # Calculate distance between points
    sampled_path = []
    dist = 0
    # Add first point
    sampled_path.append({'x': x_values[0], 'y': y_values[0], 'height': height_values[0]})
    k=1
    for point_ind in range(len(path)-1):
        p_current = np.array([path[point_ind]['x'],path[point_ind]['y'],path[point_ind]['height']])
        p_next = np.array([path[point_ind+1]['x'],path[point_ind+1]['y'],path[point_ind+1]['height']])
        dist += np.linalg.norm(p_next-p_current)
        # print(0.1*k*velocity)
        if dist>(0.1*k*velocity):
            print('estimated')
            print(0.1*k*velocity)
            print('actual')
            print(dist)
            sampled_path.append({'x': path[point_ind+1]['x'], 'y':path[point_ind+1]['y'], 'height':path[point_ind+1]['height']})
            k += 1
        

    # Convert path to numpy arrays for plotting
    path_array = np.array([(point['x'], point['y'], point['height']) for point in sampled_path])
    
    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract x, y, z coordinates
    x = path_array[:, 0]
    y = path_array[:, 1]
    z = path_array[:, 2]
    
    ax.plot(x, y, z, marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    ax.set_title('Nominal Path')
    
    plt.show()

    # Create a 3D plot
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the path
    ax.plot(x_values, y_values, height_values, 'b.-')

    # Set labels and title
    ax.set_title('3D Path with Height')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    ax.grid(True)

    # Show plot
    plt.show()

    # Print out the points
    print("Point  X       Y       Height")
    for i in range(len(lines)):
        print(f"{i+1:<7}{x_values[i]:<8.2f}{y_values[i]:<8.2f}{height_values[i]:<8.2f}")

    return path

def interpolate_points(p1, p2, num_points):
    """
    Interpolate points between p1 and p2.
    """
    x_values = np.linspace(p1['x'], p2['x'], num_points + 2)[1:-1]
    y_values = np.linspace(p1['y'], p2['y'], num_points + 2)[1:-1]
    height_values = np.linspace(p1['height'], p2['height'], num_points + 2)[1:-1]
    
    interpolated_points = [{'x': x, 'y': y, 'height': h} for x, y, h in zip(x_values, y_values, height_values)]
    
    return interpolated_points

def get_environment():
    # Define the file path
    file_path = "data/scenes/test.xyz"

    # Open the file for reading in binary mode
    with open(file_path, 'rb') as f:
        # Read the total number of points
        total_points = struct.unpack('i', f.read(4))[0]

        # Initialize a list to store point cloud data
        point_cloud = []

        # Read point cloud data
        for _ in range(total_points):
            point = np.zeros(3, dtype=np.float32)
            point[0] = struct.unpack('f', f.read(4))[0]
            point[1] = struct.unpack('f', f.read(4))[0]
            point[2] = struct.unpack('f', f.read(4))[0]
            point_cloud.append(point)

    # Convert point cloud to Open3D format
    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(point_cloud)
    point_cloud_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    test_point = np.array([11.821883,6.802271,-5.069488])
    # Reshape the array to a 3x1 vector
    test_point = test_point.reshape(3, 1)

    collision_threshold = 1
    [_, indices, distances_squared] = point_cloud_tree.search_radius_vector_3d(test_point,collision_threshold)

    # Check if any points are found within the radius
    if indices:
        # Find the closest point and its distance
        closest_distance = np.min(distances_squared)
        closest_point_index = np.argmin(distances_squared)
        closest_point = point_cloud[indices[closest_point_index]]
        
        print("Closest Point:", closest_point)
        print("Closest Distance:", np.sqrt(closest_distance))
    
    return point_cloud_tree


def metropolis_alg(nom_path,point_cloud_tree):
    # Constants
    num_paths = 10
    # variances = [2,5,10]
    variances = [0.2,0.5,1]
    variance_count = 0
    increase_int = 16000
    forward_int = 3
    max_int = 10000

    metro_paths = []
    metro_samples = []
    # Loop over number of paths
    for path_num in range(num_paths):
        print(path_num)
        start_position = nom_path[0]
        if path_num%increase_int == 0:
            variance = variances[variance_count]
            variance_count += 1

        path = []
        path.append(start_position)
        current_position = start_position
        for max_num in range(max_int):
            # Loop over a maxiumum ammount of integers
            for forward_num in range(forward_int-1):
                test_point = sample_from_normal_distribution(current_position,nom_path[forward_num],nom_path[forward_num+1],variance)
                path.append(test_point)
                current_position = test_point

            if forward_num == forward_int-2:
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

                path_to_test = path_to_test[closest_start_index:(closest_end_index+1)]


            # Calculate cost of path
            cost_percentage = calc_cost(point_cloud_tree,nominal_trajectory,path_to_test)
            random_percent = random.random()
            if cost_percentage > random_percent:
                break
            else: 
                path = []
                path_to_test = []
                path.append(start_position)
                current_position = start_position

            if max_num == max_int:
                print('Could not find path within 10000 iterations')
                path = 'no_path'


        if path != 'no_path':
            metro_paths.append(path_to_test)
            metro_samples.append(b_spline_points)

    return metro_paths,metro_samples


def quadratic_bspline(points):
    # Ensure we have exactly 3 points
    if len(points) != 3:
        raise ValueError("quadratic_bspline() requires exactly 3 points")

    # Extract x, y, z coordinates
    x = [point[0] for point in points]
    y = [point[1] for point in points]
    z = [point[2] for point in points]

    # Parameter values for interpolation
    t = np.linspace(0, 1, 6)  # 3 points + degree + 1

    # Quadratic B-spline interpolation
    bspline_x = BSpline(t, x, 2)
    bspline_y = BSpline(t, y, 2)
    bspline_z = BSpline(t, z, 2)

    return bspline_x, bspline_y, bspline_z

def sample_from_normal_distribution(current_position, nom_start, nom_next, variance):
    # Extract current position values
    x_nom, y_nom, height_nom = nom_next['x'], nom_next['y'], nom_next['height']
    x_nom_before, y_nom_before, height_nom_before = nom_start['x'], nom_start['y'], nom_start['height']
    x_start, y_start, height_start = current_position['x'], current_position['y'], current_position['height']

    x_diff = x_nom-x_nom_before
    y_diff = y_nom-y_nom_before
    height_diff = height_nom-height_nom_before

    radius_from_nom_start = np.sqrt(x_diff**2 + y_diff**2 + height_diff**2)
    theta_from_nom_start = np.arccos(height_diff / radius_from_nom_start)
    phi_from_nom_start = np.arctan2(y_diff, x_diff)

    # Create a normal distribution of theta and phi
    theta_variance = np.arcsin((variance/np.sqrt(2))/radius_from_nom_start)
    phi_variance = np.arcsin((variance/np.sqrt(2))/radius_from_nom_start)

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
    radius_to_check = 100
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

        # cost += np.transpose(test_trajectory-nominal_trajectory)*lambda_d*test_trajectory-nominal_trajectory

        if d_c > 2*r_q:
            C_collision = 0
        else:
            C_collision = (d_c**2)/(r_q**2)

        x_diff = test_trajectory[j]['x'] - nominal_trajectory[j]['x']
        y_diff = test_trajectory[j]['y'] - nominal_trajectory[j]['y']
        z_diff = test_trajectory[j]['height'] - nominal_trajectory[j]['height']
        tau = np.array([x_diff,y_diff,z_diff])
        tau = tau.reshape(3,1)

        cost += (lambda_c*C_collision + tau.T*Q*tau)*0.1

    cost_percentage = np.exp(-cost)

    return cost_percentage


def closest_obstacle(target_point, point_cloud, r_q):
    max_radius = 2 * r_q
    min_distance = float('inf')  # Initialize with infinity
    grid_size = int(max_radius * 10)  # Assuming there is a point for each 1 cm
    resolution = 0.01
    
    # Generate a grid of points within the specified radius around the target point
    x_grid, y_grid, z_grid = np.meshgrid(np.arange(-grid_size, grid_size + 1), 
                                          np.arange(-grid_size, grid_size + 1), 
                                          np.arange(-grid_size, grid_size + 1))
    grid_points = np.column_stack((x_grid.flatten() + target_point['x'], 
                                    y_grid.flatten() + target_point['y'], 
                                    z_grid.flatten() + target_point['height'])) / 10  # Convert to meters

    for obstacle_point in grid_points:
        x_round = round(obstacle_point[0], 2)
        y_round = round(obstacle_point[1], 2)
        height_round = round(obstacle_point[2], 2)

        # Find indices where rounded values match in the point cloud array
        x_index = int(x_round/resolution)
        y_index = int(y_round/resolution)
        height_index = int(height_round/resolution)
        
        if point_cloud[x_index,y_index,height_index] == 1:
            target_list = [target_point['x'], target_point['y'], target_point['height']]
            test_array = np.array(target_list)
            obstacle_list = [obstacle_point[0], obstacle_point[1], obstacle_point[2]]
            obstacle_array = np.array(obstacle_list)

            # Calculate Euclidean distance between current obstacle point and target point
            distance = np.linalg.norm(test_array - obstacle_array)
            
            # Update minimum distance if the obstacle is closer to the target point
            min_distance = min(min_distance, distance)
    
    return min_distance


def visualize_path(path, samples, nom_path=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract x, y, and height values from the path
    x_values = [point['x'] for point in path]
    y_values = [point['y'] for point in path]
    height_values = [point['height'] for point in path]

    # Plot the path
    ax.plot(x_values, y_values, height_values, label='Path')

    # Mark start point
    ax.scatter(x_values[0], y_values[0], height_values[0], c='g', marker='o', label='Start')

    # Mark end point
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

    # Show legend only for the first plot
    ax.legend()

    plt.show()


if __name__ == '__main__':
    velocity = 5 # m/s
    nominal_trajectory = get_nominal_path(velocity)
    point_cloud_tree = get_environment()
    for path_step in range(len(nominal_trajectory)-2):
        metro_paths,metro_samples = metropolis_alg(nominal_trajectory[path_step:(path_step+10)],point_cloud_tree)
        visualize_path(metro_paths[0],metro_samples[0],nominal_trajectory)
        visualize_paths(metro_paths,nominal_trajectory)
    # Chose top 3 paths