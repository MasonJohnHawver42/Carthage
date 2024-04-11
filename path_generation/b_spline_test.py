import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline

def quadratic_bspline(points):
    # Ensure we have exactly 3 points
    if len(points) != 3:
        raise ValueError("quadratic_bspline() requires exactly 3 points")

    # Extract x, y, z coordinates
    x = [point[0] for point in points]
    y = [point[1] for point in points]
    z = [point[2] for point in points]

    # Parameter values for interpolation
    t = np.linspace(0, 1, 6)  # Uniform knot vector

    # Quadratic B-spline interpolation
    bspline_x = BSpline(t, x, 2)
    bspline_y = BSpline(t, y, 2)
    bspline_z = BSpline(t, z, 2)

    return bspline_x, bspline_y, bspline_z

# Example usage
if __name__ == "__main__":
    # Example points
    points = [(1, 2, 3), (3, 1, 9), (1, 7, 3)]

    # Perform quadratic B-spline interpolation
    bspline_x, bspline_y, bspline_z = quadratic_bspline(points)

    # Evaluate the B-spline at some points
    t_eval = np.linspace(0, 1, 100)  # Evaluate over the range [0, 1] with 100 points
    interpolated_x = bspline_x(t_eval)
    interpolated_y = bspline_y(t_eval)
    interpolated_z = bspline_z(t_eval)

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot control points
    ax.scatter([point[0] for point in points], [point[1] for point in points], [point[2] for point in points], c='r', label='Control Points')

    # Plot B-spline curve
    ax.plot(interpolated_x, interpolated_y, interpolated_z, label='B-spline Curve')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title('Quadratic B-spline Curve')
    plt.legend()
    plt.show()