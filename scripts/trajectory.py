import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_route(filename):
    with open(filename, "rb") as file:
        # Read the number of rows and columns
        rows = np.fromfile(file, dtype=np.int32, count=1)[0]
        cols = np.fromfile(file, dtype=np.int32, count=1)[0]

        # Read the matrix data
        matrix_data = np.fromfile(file, dtype=np.float64, count=rows * cols)
        route_matrix = np.reshape(matrix_data, (rows, cols), order='F')

        # Read the size of the vector
        vector_size = np.fromfile(file, dtype=np.int32, count=1)[0]

        # Read the vector data
        vector_data = np.fromfile(file, dtype=np.float64, count=vector_size)
        ts_vector = np.reshape(vector_data, (vector_size,))

    return route_matrix, ts_vector

def load_trajectory(filename):
    coef_mats = []
    durations = []
    
    with open(filename, "rb") as file:
        # Read the number of trajectory pieces
        piece_n = np.fromfile(file, dtype=np.int32, count=1)[0]

        for _ in range(piece_n):
            # Read the number of rows and columns of the coefficient matrix
            rows = np.fromfile(file, dtype=np.int32, count=1)[0]
            cols = np.fromfile(file, dtype=np.int32, count=1)[0]

            # Read the duration of the trajectory piece
            duration = np.fromfile(file, dtype=np.float64, count=1)[0]
            durations.append(duration)

            # Read the coefficient matrix data
            matrix_data = np.fromfile(file, dtype=np.float64, count=rows * cols)
            coef_matrix = np.reshape(matrix_data, (rows, cols), order='F')

            # Store the trajectory piece
            coef_mats.append(coef_matrix)

    return coef_mats, np.array(durations)

# Traj Utility

def locate_piece_idx(t, durations):
    N = len(durations)
    idx = 0
    for idx in range(N):
        dur = durations[idx]
        if t <= dur:
            break
        t -= dur
    else:
        idx = N - 1
        t += durations[idx]
    return t, idx

def polyder(t, k = 0, order = 10):
    if k == 'all':
        terms = np.array([polyder(t,k,order) for k in range(1,5)])
    else:
        terms = np.zeros(order)
        coeffs = np.polyder([1]*order,k)[::-1]
        pows = t**np.arange(0,order-k,1)
        terms[k:] = coeffs*pows
    return terms[::-1]

def sample_traj(t, k, coef_mats, ts):
    """
        Samples the Trajectory at time t for the kth derivative
        .i.e k = 0 is the position, k = 1 is the velocity and so on

        coef_mats and ts define the trajectory and are loaded in with load_trajectory

        Assumptions:
          0 <= t <= sum(ts)
          0 <= k <= 7
    """
    t, idx = locate_piece_idx(t, ts)
    return coef_mats[idx] @ polyder(t / ts[idx], k, 8)

# Load data
route, ts_rt = load_route("../data/tmp/route.rt")
coef_mats, ts_tj = load_trajectory("../data/tmp/traj.traj")

# Sample Trajectory
N = 100
times = np.linspace(0, sum(ts_tj), N)
smooth_pos = np.array([sample_traj(t, 0, coef_mats, ts_tj) for t in times])

# plot route (waypoints) / trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(route[0, :], route[1, :], route[2, :])
ax.plot(smooth_pos[:, 0], smooth_pos[:, 1], smooth_pos[:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Route Plot')
plt.show()