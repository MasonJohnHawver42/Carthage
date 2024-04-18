import numpy as np
import struct

from tqdm import tqdm

import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt


def voxelize(pt):
    return [ int(v * (1 << max_depth)) for v in pt]

with open("data/scenes/test.xyz", 'rb') as f:
    num_points = struct.unpack('i', f.read(4))[0]
    # num_points = min(4, num_points)
    points = []
    for _ in range(num_points):
        x = struct.unpack('f', f.read(4))[0]
        y = struct.unpack('f', f.read(4))[0]
        z = struct.unpack('f', f.read(4))[0]
        points.append([x, y, z])
    points = np.array(points)

# Read the bounding box from test.bb
with open("data/scenes/test.bb", 'rb') as f:
    bounding_box = []
    for _ in range(2):  # Assuming there are 2 points in the bounding box
        x = struct.unpack('f', f.read(4))[0]
        y = struct.unpack('f', f.read(4))[0]
        z = struct.unpack('f', f.read(4))[0]
        bounding_box.append([x, y, z])
    bounding_box = np.array(bounding_box)

max_depth = 8

n_points = (points - bounding_box[0]) / (bounding_box[1] - bounding_box[0]).max()

p = (bounding_box[1] - bounding_box[0]) / (bounding_box[1] - bounding_box[0]).max()
size = np.ceil((2 ** max_depth) * p).astype(int)

print(size)
print(n_points)

grid = np.zeros(size)

for point in n_points:
    vox = voxelize(point)
    grid[vox[0], vox[1], vox[2]] = 1

obstacles = grid != 0
distance_map = distance_transform_edt(~obstacles)

print(distance_map.shape, (2 ** max_depth), ((2 ** max_depth) ** 3) * 4, size[0] * size[1] * size[2] * 4)

# save map

with open("data/scenes/test.ff", 'wb') as f:
    f.write(struct.pack('I', max_depth))
    f.write(struct.pack('I', size[0]))
    f.write(struct.pack('I', size[1]))
    f.write(struct.pack('I', size[2]))

    for x in range(size[0]):
        for y in range(size[1]):
            for z in range(size[2]):
                f.write(struct.pack('f', distance_map[x, y, z]))
    
    f.write(struct.pack('f', bounding_box[0, 0]))
    f.write(struct.pack('f', bounding_box[0, 1]))
    f.write(struct.pack('f', bounding_box[0, 2]))
    f.write(struct.pack('f', bounding_box[1, 0]))
    f.write(struct.pack('f', bounding_box[1, 1]))
    f.write(struct.pack('f', bounding_box[1, 2]))

print("plot")

jump = (size[1] // 30)
for i in range(30):
    plt.subplot(3, 10, i + 1)
    plt.imshow(distance_map[:, i * jump, :])
    plt.axis('off')
plt.show()

exit()

# grid = np.zeros((n, n))
# grid[n//2, n//2] = 1

print(grid)
# plt.imshow(grid)
# plt.show()

result = np.zeros((n, n, 2))

left = np.array([0, -1], dtype=int)
up = np.array([-1, 0], dtype=int)
dirs = [left, up]

for i in range(n):
    for j in range(n):
        if grid[i, j] == 1:
            continue

        for d in dirs:
            if 0 <= d[0] + i < n and 0 <= d[1] + j < n:
                res = result[d[0] + i, d[1] + j, :] + d
                dist = (res[0] ** 2) + (res[1] ** 2)
                if grid[d[0] + i, d[1] + j] != 0 and (grid[i, j] == 0 or ((result[i, j, 0] ** 2) + (result[i, j, 1] ** 2)) > dist):
                    result[i, j, :] = res
                    grid[i, j] = 2
                

right = np.array([0, 1], dtype=int)
down = np.array([1, 0], dtype=int)
dirs = [right, down]

for i in range(n)[::-1]:
    for j in range(n)[::-1]:
        if grid[i, j] == 1:
            continue

        for d in dirs:
            if 0 <= d[0] + i < n and 0 <= d[1] + j < n:
                res = result[d[0] + i, d[1] + j, :] + d
                dist = (res[0] ** 2) + (res[1] ** 2)
                if grid[d[0] + i, d[1] + j] != 0 and (grid[i, j] == 0 or ((result[i, j, 0] ** 2) + (result[i, j, 1] ** 2)) > dist):
                    result[i, j, :] = res
                    grid[i, j] = 2

dirs = [left, up]

for i in range(n):
    for j in range(n):
        if grid[i, j] == 1:
            continue

        for d in dirs:
            if 0 <= d[0] + i < n and 0 <= d[1] + j < n:
                res = result[d[0] + i, d[1] + j, :] + d
                dist = (res[0] ** 2) + (res[1] ** 2)
                if grid[d[0] + i, d[1] + j] != 0 and (grid[i, j] == 0 or ((result[i, j, 0] ** 2) + (result[i, j, 1] ** 2)) > dist):
                    result[i, j, :] = res
                    grid[i, j] = 2 

dirs = [right, down]

for i in range(n)[::-1]:
    for j in range(n)[::-1]:
        if grid[i, j] == 1:
            continue

        for d in dirs:
            if 0 <= d[0] + i < n and 0 <= d[1] + j < n:
                res = result[d[0] + i, d[1] + j, :] + d
                dist = (res[0] ** 2) + (res[1] ** 2)
                if grid[d[0] + i, d[1] + j] != 0 and (grid[i, j] == 0 or ((result[i, j, 0] ** 2) + (result[i, j, 1] ** 2)) > dist):
                    result[i, j, :] = res
                    grid[i, j] = 2


# head = np.zeros((n, n, 2))

# light_fifo = []

# for i in range(n):
#     for j in range(n):
#         if grid[i][j] == 1:
#             light_fifo.append(np.array([i, j]))


# dirs = [np.array([1, 0]), np.array([-1, 0]), np.array([0, 1]), np.array([0, -1])]

# while len(light_fifo) != 0:
#     pos = light_fifo.pop(0)

#     for d in dirs:
#         tmp = pos + d
#         if 0 <= tmp[0] < n and 0 <= tmp[1] < n:
#             res = result[pos[0], pos[1], :] + d
#             dist = res[0] ** 2 + res[1] ** 2
#             if grid[tmp[0], tmp[1]] == 0:
#                 result[tmp[0], tmp[1], :] = res
#                 grid[tmp[0], tmp[1]] = 2
#                 head[tmp[0], tmp[1], :] = tmp
#                 light_fifo.append(tmp)

#             if grid[tmp[0], tmp[1]] == 2 and ((result[tmp[0], tmp[1], 0] ** 2) + (result[tmp[0], tmp[1], 1] ** 2)) > dist:
#                 result[tmp[0], tmp[1], :] = res
#                 old = head[tmp[0], tmp[1], :]
#                 head[tmp[0], tmp[1], :] = tmp
#                 light_fifo.append(tmp)

#                 light_fifo = [h for h in light_fifo if not np.array_equal(h, old)]


rc = np.sqrt(result[:, :, 0] ** 2 + result[:, :, 1] ** 2).clip(0, 32)
# print(result)
# rc = (np.abs(result[:, :, 0]) + np.abs(result[:, :, 1])).clip(0, 32)
rc[grid == 1] = 32
plt.imshow(rc)
plt.show()
