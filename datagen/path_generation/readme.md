# CV Project : carthage

**Author**: Marcus Berg \
**Email**: ber00256@umn.edu

## Summary
The following program performs Metropolis Hastings Sampling on a reference trajectory, chose the 3 paths associated with the lowest cost at each time step, and creates a dataset. An odometry path, a reference apth, a pointcloud, and images from the drone at each timestep of the odometry is required.

## Compile and run program
--path_to_csv is a required argument, which is a csv file cotaining of 
odometry_path, reference_path, pointcloud_path, img_dir.
odometry path is a path to an odometry file.
reference_path is a path to a reference trajectory.
point_cloud_path is a path to a ply file.
img_dir is path to an image directory with depth images from the drone at each sampled step of the odometry.

--visualize is a voluntary argument to visualize the sampling of the bspline at each time step, 
and the three best trajectories at each time step