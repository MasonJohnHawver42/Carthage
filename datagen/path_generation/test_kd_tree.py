import numpy as np
import open3d as o3d

# Create a point cloud
point_cloud = np.array([
    [0, 0, 0],
    [1, 1, 1],
    [2, 2, 2],
    [3, 3, 3],
    [4, 4, 4]
], dtype=np.float32)

# Convert point cloud to Open3D format
point_cloud_o3d = o3d.geometry.PointCloud()
point_cloud_o3d.points = o3d.utility.Vector3dVector(point_cloud)

# Create KDTree
tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

# Define the search point
search_point = np.array([2.5, 2.5, 2.5], dtype=np.float32)

# Define the search radius
radius = 1.0

try:
    # Search points within the radius
    [_, indices, distances_squared] = tree.search_radius_vector_3d(search_point, radius)

    # Check if any points are found within the radius
    if indices:
        # Find the closest point and its distance
        closest_distance = np.min(distances_squared)
        closest_point_index = np.argmin(distances_squared)
        closest_point = point_cloud[indices[closest_point_index]]
        
        print("Closest Point:", closest_point)
        print("Closest Distance:", np.sqrt(closest_distance))
    else:
        print("No points found within the radius.")

except Exception as e:
    print(f"Error calling search_radius_vector_3d(): {e}")

distance = np.sqrt((2 - 2.5)**2 + (2 - 2.5)**2 + (2 - 2.5)**2)
print("Expected Closest Distance:", distance)