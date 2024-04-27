import numpy as np
import struct
import sys

from scipy.ndimage import distance_transform_edt

class Frame:
    def __init__(self):
        self.children = None
        self.state = 0
        # 0 - empty | 1 - deprecated | 2 - frame | 3 - grid

class Grid:
    def __init__(self):
        self.occupancy = [255] * 32 * 32  * 32
        # 0 - empty | 1 - full

def voxelize(pt):
    return [ int(v * (1 << max_depth)) for v in pt]

def add_voxel(voxel, value):
    
    frame = root
    v = max(min(int(value), 255), 0) 

    for i in range(frame_depth):
        n = grid_depth + (frame_depth - i - 1)
        mask = 1 << n
        child = (((voxel[0] & mask) >> n) << 2) | (((voxel[1] & mask) >> n) << 1) | ((voxel[2] & mask) >> n)

        if frame.state == 0:
            frame.children = [Frame() for _ in range(8)]
            frame.state = 2
        
        if frame.state == 2:
            frame = frame.children[child]
        else:
            raise ValueError

    if frame.state == 0:
        frame.children = Grid()
        frame.state = 3
    
    grid = frame.children
    
    mask = (1 << grid_depth) - 1
    index = ((voxel[0] & mask) << 10) | ((voxel[1] & mask) << 5) | ((voxel[2] & mask))

    grid.occupancy[index] = v

def optimize_sister(parent, child):

    curr = parent.children[child]
    cnt_1 = sum([ 1 if s == 2 else 0 for s in curr.states])
    cnt_2 = sum([ 1 if s == 0 else 0 for s in curr.states])
    
    if cnt_1 == 4:
        curr.children = []
        parent.states[child] = 2
    
    if cnt_2 == 4:
        curr.children = []
        parent.states[child] = 3

def optimize(curr):

    if curr.state == 0:
        return

    if curr.state == 2:
        for i in range(8):
            optimize(curr.children[i])

        uniform = all(child.state == 0 for child in curr.children)
        uniform = False if not uniform else all((child.children == curr.children[0].children for child in curr.children))
        # empty_sum = sum([ 1 if child.state == 0 else 0 for child in curr.children])
        # full_sum = sum([ 1 if child.state == 1 else 0 for child in curr.children])
        
        if uniform:
            curr.children = curr.children[0].children
            curr.state = 0
    
    if curr.state == 3:
        grid = curr.children

        uniform = all(o == grid.occupancy[0] for o in grid.occupancy)
        if uniform:
            curr.children = grid.occupancy[0]
            curr.state = 0

def convert(frame):
    if frame.state == 0 or frame.state == 1:
        return

    global frames, grids

    if frame.state == 2:
        index = len(frames)
        frames += frame.children
        for i in range(8):
            convert(frame.children[i])
        frame.children = index
    
    if frame.state == 3:
        index = len(grids)
        grids.append(frame.children)
        frame.children = index


if len(sys.argv) != 2:
    print("Usage: python3 scripts/octree.py <scene file>")
    exit(0)

scn_fn = sys.argv[1]
scn_bb = ".".join(sys.argv[1].split(".")[:-1]) + ".bb"
scn_xyz = ".".join(sys.argv[1].split(".")[:-1]) + ".xyz"
scn_oct = ".".join(sys.argv[1].split(".")[:-1]) + ".oct"

fn_scn = sys.argv[1]

with open(scn_xyz, 'rb') as f:
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
with open(scn_bb, 'rb') as f:
    bounding_box = []
    for _ in range(2):  # Assuming there are 2 points in the bounding box
        x = struct.unpack('f', f.read(4))[0]
        y = struct.unpack('f', f.read(4))[0]
        z = struct.unpack('f', f.read(4))[0]
        bounding_box.append([x, y, z])
    bounding_box = np.array(bounding_box)

n_points = (points - bounding_box[0]) / (bounding_box[1] - bounding_box[0]).max()

root = Frame()

frames = [root]
grids = []

frame_depth = 3
grid_depth = 5
max_depth = frame_depth + grid_depth

p = (bounding_box[1] - bounding_box[0]) / (bounding_box[1] - bounding_box[0]).max()
size = np.ceil((2 ** frame_depth) * p).astype(int) * 32

grid = np.zeros(size)

for point in n_points:
    vox = voxelize(point)
    grid[vox[0], vox[1], vox[2]] = 1

obstacles = grid != 0
distance_map = distance_transform_edt(~obstacles)

for x in range(size[0]):
    for y in range(size[1]):
        for z in range(size[2]):
            add_voxel([x, y, z], distance_map[x, y, z] ** 2)

# print(size)

# for i in range(n_points.shape[0]):
#     vox = voxelize(n_points[i, :])
#     add_voxel(vox, 1)  

# optimize(root)
convert(root)

print(len(frames), len(frames) * 5)
print(len(grids), len(grids) * 32 * 32 * 32, len(frames) * 5 + len(grids) * 32 * 32 * 32)
print(scn_oct)

with open(scn_oct, 'wb') as f:
    f.write(struct.pack('I', len(frames)))
    f.write(struct.pack('I', len(grids)))
    f.write(struct.pack('I', frame_depth))
    f.write(struct.pack('I', grid_depth))

    f.write(struct.pack('f', bounding_box[0, 0]))
    f.write(struct.pack('f', bounding_box[0, 1]))
    f.write(struct.pack('f', bounding_box[0, 2]))
    f.write(struct.pack('f', bounding_box[1, 0]))
    f.write(struct.pack('f', bounding_box[1, 1]))
    f.write(struct.pack('f', bounding_box[1, 2]))

    for frame in frames:
        f.write(struct.pack('I', frame.children if frame.state == 2 or frame.state == 3 else 0))
        f.write(struct.pack('I', frame.state))
    
    for grid in grids:
        for i in range(32 * 32 * 32):
            f.write(struct.pack('B', grid.occupancy[i]))
