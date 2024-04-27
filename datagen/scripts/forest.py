import numpy as np
import matplotlib.pyplot as plt
import noise

from scipy.stats import qmc
from scipy.stats import beta
from scipy.ndimage import sobel, gaussian_filter

def generate_vertices(heightmap, normal_map, heightmap_size, size=2, origin=(0, 0, 0)):
    vertices = []
    normals = []
    uvs = []

    # We need to calculate the step between vertices
    step_x = size / (heightmap_size[0] - 1)
    step_y = size / (heightmap_size[1] - 1)

    for x in range(heightmap_size[0]):
        for y in range(heightmap_size[1]):
            x_coord = origin[0] + step_x * x
            y_coord = origin[1] + step_y * y
            z_coord = origin[2] + heightmap[x][y]
            vertices.append((x_coord, y_coord, z_coord))

            # Calculate normals from normal map
            normal = normal_map[x][y]
            normals.append(normal)

            # Calculate UVs
            u = x / (heightmap_size[0] - 1)
            v = y / (heightmap_size[1] - 1)
            uvs.append((u, v))

    return vertices, normals, uvs

def generate_tris(grid_size):
    tris = []
    for x in range(grid_size[0]-1):
        for y in range(grid_size[1]-1):
            index = x*grid_size[0]+y
            a = index
            b = index+1
            c = index+grid_size[0]+1
            d = index+grid_size[0]
            tris.append((c, b, a))
            tris.append((d, c, a))
    return tris

def export_obj(vertices, tris, normals, uvs, filename_obj):
    # Extract filename without extension for material name

    # Export OBJ file
    with open(filename_obj, "w") as obj_file:
        obj_file.write("mtllib ./" + filename_obj.split("/")[-1].split(".")[0] + ".mtl\n")  # Reference to MTL file
        for vertex in vertices:
            obj_file.write("v " + " ".join(map(str, vertex)) + "\n")
        for uv in uvs:
            obj_file.write("vt " + " ".join(map(str, uv)) + "\n")
        for normal in normals:
            obj_file.write("vn " + " ".join(map(str, normal)) + "\n")
        obj_file.write("usemtl None\n")  # Use material defined in MTL file
        for tri in tris:
            obj_file.write("f " + " ".join([f"{tri[i]+1}/{tri[i]+1}/{tri[i]+1}" for i in range(3)]) + "\n")

    # Export MTL file
    with open(".".join(filename_obj.split(".")[:-1]) + ".mtl", "w") as mtl_file:
        mtl_file.write("newmtl None\n")
        mtl_file.write("Ka 0.0 0.8 0.1\n")  # Ambient color
        mtl_file.write("Kd 1.0 1.0 1.0\n")  # Diffuse color
        mtl_file.write("Ks 0.0 0.0 0.0\n")  # Specular color
        mtl_file.write("Ns 0.0\n")           # Shininess

def generate_perlin_noise_2d(shape, res, scale):
    noise_array = np.zeros(shape)
    seed = [np.random.randint(0,100) for i in range(len(res))]
    for i in range(shape[0]):
        for j in range(shape[1]):
            for r, s, d in zip(res, scale, seed):
                noise_array[i][j] += noise.pnoise2(i/r, 
                                                   j/r, 
                                                   repeatx=shape[0], 
                                                   repeaty=shape[1], base=d) * s



    return noise_array

def generate_normal_map(heightmap, size):
    normal_map = np.zeros((*heightmap.shape, 3))
    height, width = heightmap.shape

    for y in range(height):
        for x in range(width):
            dx = (heightmap[min(y + 1, height - 1), x] - heightmap[max(y - 1, 0), x]) * (size / shape)
            dy = (heightmap[y, min(x + 1, width - 1)] - heightmap[y, max(x - 1, 0)]) * (size / shape)
            normal = np.array([-dx, -dy, 1.0])
            normal /= np.linalg.norm(normal)
            normal_map[y, x] = normal

    return normal_map


def make_terrain(obj_fn, shape=512, size=50, tree_dist=5, height=10):

    origin = (-size/2, -size/2, 0)

    noise = generate_perlin_noise_2d((shape, shape), [shape / 2, shape / 8, shape / 32, shape / 64], [height / 2, height / 8, height / 64, height / 128])

    rng = np.random.default_rng()
    engine = qmc.PoissonDisk(d=2, radius=tree_dist / size, seed=rng)
    sample = engine.random(1000) * shape

    gradient_x = sobel(noise, axis=0)
    gradient_y = sobel(noise, axis=1)
    gradient_magnitude = np.sqrt(gradient_x**2 + gradient_y**2)

    chosen_samples = []
    res_samples = []

    for point in sample:
        x, y = point
        x_idx = int(x)
        y_idx = int(y)
        if gradient_magnitude[x_idx, y_idx] < 3:
            chosen_samples.append(point)
            res_samples.append((point[0] * (size / shape) + origin[0], point[1] * (size / shape) + origin[1], noise[x_idx, y_idx] + origin[2]))

    chosen_samples = np.array(chosen_samples)
    res_samples = np.array(res_samples)

    x = np.linspace(0, shape, shape) * (size / shape) + origin[0]
    y = np.linspace(0, shape, shape) * (size / shape) + origin[1]
    X, Y = np.meshgrid(x, y)
    Z = noise + origin[2]

    dx = sobel(Z, axis=0)
    dy = sobel(Z, axis=1)
    dz = np.ones_like(noise)

    magnitude = np.sqrt(dx**2 + dy**2 + dz**2)
    dx /= magnitude
    dy /= magnitude
    dz /= magnitude

    normal_map = np.stack((dx, dy, dz), axis=-1)

    vertices, normals, uvs = generate_vertices(noise, normal_map, (shape, shape), size=size, origin=origin)
    tris = generate_tris((shape, shape))
    export_obj(vertices, tris, normals, uvs, obj_fn)

    return res_samples


# plt.imshow(noise)
# plt.colorbar()
# plt.scatter(chosen_samples[:, 0], chosen_samples[:, 1], label='Points')
# plt.colorbar()
# plt.show()

# fig, axs = plt.subplots(1, 3, figsize=(15, 5))

# for i, component in enumerate(['X', 'Y', 'Z']):
#     axs[i].imshow(normal_map[:, :, i], cmap='gray')
#     axs[i].set_title(f'Normal {component}')

# plt.tight_layout()
# plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot_surface(X, Y, Z)

# ax.set_zlim(-size/2, size/2)

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Height')

# plt.show()