import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import random
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline

def get_nominal_path():
    # Define start and end points
    start_point = np.array([2.5, 2.5, 3])
    end_point = np.array([7.5, 7.5, 4])

    # Generate straight line path points
    num_points = 20
    x_values = np.linspace(start_point[0],end_point[0],num_points)
    y_values = np.linspace(start_point[1],end_point[1],num_points)
    height_values = np.linspace(start_point[2],end_point[2],num_points)

    # Plot the path
    plt.figure(figsize=(8, 6))
    plt.plot(x_values, y_values, 'b.-')
    plt.title('Straight Line Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    # Print out the points
    print("Point   X   Y   Height")
    for i in range(num_points):
        print(f"{i+1:<7}{x_values[i]:<5.2f}{y_values[i]:<5.2f}{height_values[i]:<8.2f}")

    # Create a list of dictionaries representing points
    path = []
    for i in range(num_points):
        point = {
            'x': x_values[i],
            'y': y_values[i],
            'height': height_values[i],
        }
        path.append(point)

    return path


def get_environment():
    # Define ranges and resolution
    x_range = (0, 10)  # x range from 0 to 10 meters
    y_range = (0, 10)  # y range from 0 to 10 meters
    z_range = (0, 5)   # z range (height) from 0 to 5 meters
    resolution_cm = 10  # Resolution of 1 cm

    # Generate the point cloud
    point_cloud = generate_point_cloud(x_range, y_range, z_range, resolution_cm)

    return point_cloud


def generate_point_cloud(x_range, y_range, z_range, obstacle_density):
    # Calculate number of points in each dimension
    num_points_x = int((x_range[1] - x_range[0]) * 100) + 1
    num_points_y = int((y_range[1] - y_range[0]) * 100) + 1
    num_points_z = int((z_range[1] - z_range[0]) * 100) + 1

    # Generate grid of points
    x_values = np.linspace(x_range[0], x_range[1], num_points_x)
    y_values = np.linspace(y_range[0], y_range[1], num_points_y)
    z_values = np.linspace(z_range[0], z_range[1], num_points_z)

    # Create a meshgrid of x, y, z points
    xx, yy, zz = np.meshgrid(x_values, y_values, z_values, indexing='ij')

    # Create point cloud with zeros representing no obstacles
    point_cloud = np.zeros_like(xx, dtype=int)

    # Generate obstacle positions
    obstacle_positions = []
    num_obstacles = int((x_range[1] - x_range[0]) * (y_range[1] - y_range[0]) * (z_range[1] - z_range[0]) * obstacle_density)
    for _ in range(num_obstacles):
        obstacle_x = np.random.uniform(x_range[0], x_range[1])
        obstacle_y = np.random.uniform(y_range[0], y_range[1])
        obstacle_z = np.random.uniform(z_range[0], z_range[1])
        obstacle_positions.append((obstacle_x, obstacle_y, obstacle_z))

    # Set ones at obstacle positions
    for obstacle_position in obstacle_positions:
        obstacle_x, obstacle_y, obstacle_z = obstacle_position
        # Convert obstacle positions to grid indices
        obstacle_x_index = np.argmin(np.abs(x_values - obstacle_x))
        obstacle_y_index = np.argmin(np.abs(y_values - obstacle_y))
        obstacle_z_index = np.argmin(np.abs(z_values - obstacle_z))
        # Set obstacle position to one in the point cloud
        point_cloud[obstacle_x_index, obstacle_y_index, obstacle_z_index] = 1

    return point_cloud


def metropolis_alg(nom_path,environment):
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
        for max_num in range(max_int):
            # Loop over a maxiumum ammount of integers
            for forward_num in range(forward_int-1):
                test_point = sample_from_normal_distribution(nom_path[forward_num+1],variance)
                path.append(test_point)

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
            # cost = calc_cost(environment,ref_point,test_point)
            # random_percent = random.random()
            cost = 0.75
            random_percent = 0.5
            if cost > random_percent:
                break

            else: 
                path = []
                path_to_test = []
                
                path.append(start_position)

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

def sample_from_normal_distribution(nom_point, variance):
    # Extract current position values
    x_nom, y_nom, height_nom = nom_point['x'], nom_point['y'], nom_point['height']

    radius_nom = np.sqrt(x_nom**2 + y_nom**2 + height_nom**2)
    theta_nom = np.arccos(height_nom / radius_nom)
    phi_nom = np.arctan2(y_nom, x_nom)

    # Create a normal distribution of theta and phi
    theta_variance = np.arcsin((variance/np.sqrt(2))/radius_nom)
    phi_variance = np.arcsin((variance/np.sqrt(2))/radius_nom)

    # Generate random offsets for theta and phi
    theta_offset = np.random.normal(loc=0, scale=theta_variance)
    phi_offset = np.random.normal(loc=0, scale=phi_variance)

    # New phi and theta coordinates
    new_theta = theta_nom + theta_offset
    new_phi = phi_nom + phi_offset

    # Write back to xyz coordinates
    new_x = radius_nom * np.sin(new_theta) * np.cos(new_phi)
    new_y = radius_nom * np.sin(new_theta) * np.sin(new_phi)
    new_z = radius_nom * np.cos(new_theta)

    # Create and return the new point
    test_point = {
        'x': new_x,
        'y': new_y,
        'height': new_z,
    }

    return test_point
    

def calc_cost(environment,ref_point,test_point):
    r_q = 0.2
    lambda_c = 100
    lambda_d = 1
    d_c = closest_obstacle(test_point,environment,r_q)
    if d_c > 2*r_q:
        C_collision = 0
    else:
        C_collision = -(d_c**2)/(r_q**2)
    eucliedean_distance = calc_euc_dist(test_point,ref_point)
    cost = lambda_c*C_collision + lambda_d*eucliedean_distance
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


def calc_euc_dist(test_point,ref_point):
    test_list = [test_point['x'],test_point['y'],test_point['height']]
    test_array = np.array(test_list)
    ref_list = [ref_point['x'],ref_point['y'],ref_point['height']]
    ref_array = np.array(ref_list)
    eucliedean_distance = np.linalg.norm(test_array - ref_array)

    return eucliedean_distance


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
    nom_path = get_nominal_path()
    environment = get_environment()
    for path_step in range(len(nom_path)-2):
        metro_paths,metro_samples = metropolis_alg(nom_path[path_step:(path_step+10)],environment)
        visualize_path(metro_paths[0],metro_samples[0],nom_path)
        visualize_paths(metro_paths,nom_path)
    # Chose top 3 paths