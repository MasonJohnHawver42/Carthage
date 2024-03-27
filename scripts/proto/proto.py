import matplotlib.pyplot as plt
import random
import math

class Node:
    def __init__(self):
        self.children = []
        self.states = [0, 0, 0, 0]



def voxelize(pt):
    voxel = []
    for i in range(2):
        voxel.append(int(pt[i] * (1 << max_depth)))
    return voxel

def add_voxel(voxel):
    curr = root

    for i in range(max_depth):
        n = max_depth - i - 1
        mask = 1 << n
        child = (((voxel[0] & mask) >> n) << 1) | ((voxel[1] & mask) >> n)
        # print(child, voxel[0] & mask, n + 1)
        curr.states[child] = 1 if i + 1 < max_depth else 2

        if i + 1 < max_depth:
            if len(curr.children) == 0:
                for j in range(4):
                    new_node = Node()
                    curr.children.append(new_node)
                
            curr = curr.children[child]


f_xp = [(1, 0), (1, 1)]
f_xn = [(0, 0), (0, 1)]
f_yp = [(0, 1), (1, 1)]
f_yn = [(0, 0), (1, 0)]
faces = [f_xn, f_yn, f_xp, f_yp]

def walk(p_voxel, p_depth, curr):

    start = [(v << (max_depth - p_depth)) / float(1 << max_depth) for v in p_voxel]
    half_size = (1.0 / float(1 << p_depth)) / 2.0
    center = [v + half_size for v in start]

    # if sum([1 if curr.states[i] == 2 or curr.states[i] == 1 else 0 for i in range(4)]) == 4:
    #     color = (0, 1, 0)

    # plt.plot([center[0] - half_size, center[0] + half_size], [center[1], center[1]], "b-")
    # plt.plot([center[0], center[0]], [center[1] - half_size, center[1] + half_size], "b-")

    voxel = [ v << 1 for v in p_voxel ]
    depth = p_depth + 1

    for i in range(4):
        voxel[0] = (voxel[0] & (~1)) | (((i & 0x2) >> 1) & 1); 
        voxel[1] = (voxel[1] & (~1)) | ((i & 0x1) & 1);

        # print("{}: [{}, {}] {}".format(depth, voxel[0], voxel[1], curr.states[i] == 1))
        
        if (curr.states[i] == 2):

            start = [(v << (max_depth - depth)) / float(1 << max_depth) for v in voxel]
            half_size = (1.0 / float(1 << depth)) / 2.0
            center = [v + half_size for v in start]

            square_coords = [
                (center[0] - half_size, center[1] - half_size),
                (center[0] + half_size, center[1] - half_size),
                (center[0] + half_size, center[1] + half_size),
                (center[0] - half_size, center[1] + half_size),
                (center[0] - half_size, center[1] - half_size)  # Closing the square
            ]

            # Plot the filled square
            rand_color = (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
            plt.fill(*zip(*square_coords), color=rand_color)

            # square_coords = []

            # for j in range(4):
            #     axis = j & 1
            #     dir = (j & 2) - 1
            #     n_vox, valid = nbr(voxel, depth, axis, dir)
                
            #     v_face = [((pt[0] * 2 * half_size) + start[0], (pt[1] * 2 * half_size) + start[1]) for pt in faces[axis + dir + 1]]

            #     if not valid:
            #         plt.plot(*zip(*v_face), 'r-')
            #         continue
                
            #     state, frame = get_state(n_vox, depth)
            #     fs = face_state(state, frame, axis, -1 * dir)

            #     if not fs:
            #         plt.plot(*zip(*v_face), 'r-')

        if (len(curr.children) != 0 and curr.states[i] != 2 and curr.states[i] != 3):
            walk(voxel, depth, curr.children[i])


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

    if len(curr.children) == 0:
        return

    for i in range(4):
        optimize(curr.children[i])
        optimize_sister(curr, i)


def count(p_voxel, p_depth, curr):
    voxel = [ v << 1 for v in p_voxel ]
    depth = p_depth + 1
    
    cnt = 4

    for i in range(4):
        voxel[0] = (voxel[0] & (~1)) | (((i & 0x2) >> 1) & 1); 
        voxel[1] = (voxel[1] & (~1)) | ((i & 0x1) & 1);

        if len(curr.children) != 0 and curr.states[i] != 2 and curr.states[i] != 3:
            cnt += count(voxel, depth, curr.children[i])
    
    return cnt

def nbr(p_voxel, p_depth, axis, dir):
    voxel = [v for v in p_voxel]
    voxel[axis] += dir;
    return voxel, 0 <= voxel[axis] <= (1 << p_depth)

x_pos = [0b10, 0b11]
x_neg = [0b00, 0b01]
y_pos = [0b01, 0b11]
y_neg = [0b00, 0b10]
faces_dirs = [x_neg, y_neg, x_pos, y_pos]

def get_state(voxel, depth):
    i = 0
    curr = root
    while i + 1 < depth and len(curr.children) != 0:
        n = depth - i - 1
        mask = 1 << n
        child = (((voxel[0] & mask) >> n) << 1) | ((voxel[1] & mask) >> n)
        curr = curr.children[child]
        i += 1

    n = depth - i - 1
    mask = 1 << n
    child = (((voxel[0] & mask) >> n) << 1) | ((voxel[1] & mask) >> n)

    if len(curr.children) == 0:
        return curr.states[child], None
    
    return curr.states[child], curr.children[child]


def face_state(state, frame, axis, dir):
    if frame is None:
        return False if state == 0 else True
    
    res = True
    
    for i in faces_dirs[axis + dir + 1]:
        if len(frame.children) != 0:
            if not face_state(frame.states[i], frame.children[i], axis, dir):
                res = False

        elif frame.states[i] == 0:
            res = False
    
    return res


def circle_pts(r, d, n, pos):

    pts = []

    for i in range(n):
        theta = random.uniform(0, math.pi * 2)
        r_rand = r + random.uniform(-d / 2.0, d / 2.0)
        x = (r_rand * math.cos(theta)) + pos[0]
        y = (r_rand * math.sin(theta)) + pos[1]
        pts.append((x, y))
    
    return pts

# for i in range(1, 10):

root = Node()
max_depth = 6

pts_1 = circle_pts(.4 / 2.0, .5, 10**(max(max_depth - 1, 3)), [.5, .5])
pts_2 = circle_pts(.01 / 2.0, .1, 10**(max(max_depth - 1, 3)), [.375, .625])
pts_3 = circle_pts(.01 / 2.0, .1, 10**(max(max_depth - 1, 3)), [.625, .625])
pts_4 = circle_pts(.01 / 2.0, .1, 10**(max(max_depth - 1, 3)), [.5, .375])


pts = pts_1 + pts_2 + pts_3 + pts_4
for pt in pts:
    voxel = voxelize(pt)
    # print(pt, voxel)
    # for v in voxel:
    #     print(bin(v))
    add_voxel(voxel)

print("inited")
optimize(root)
print("optimized")
print("{}: Nodes: {}".format(max_depth, count([0, 0], 0, root)))

# plt.subplot(3, 3, i)

x = [0, 1, 1, 0, 0]
y = [0, 0, 1, 1, 0]

plt.plot(x, y, 'b-')

walk([0, 0], 0, root)

x, y = zip(*pts)
# plt.scatter(x, y, marker='o', color='yellow')

plt.show()