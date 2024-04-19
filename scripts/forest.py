import numpy as np
import matplotlib.pyplot as plt
import noise

def generate_vertices(heightmap, heightmap_size):
    vertices = []

    # The origin and size of mesh
    origin = (-1, -0.75, -1)
    size = 2
    max_height = 0.5

    # We need to calculate the step between vertices 
    step_x = size/(heightmap_size[0]-1)
    step_y = size/(heightmap_size[1]-1)

    for x in range(heightmap_size[0]):
        for y in range(heightmap_size[1]):
            x_coord = base[0] + step_x*x 
            y_coord = base[1] + max_height*heightmap[x][y]
            z_coord = base[2] + step_y*y
            vertices.append((x_coord, y_coord, z_coord))
    return vertices

def generate_tris(grid_size):
    tris = []
    for x in range(grid_size[0]-1):
        for y in range(grid_size[1]-1):
            index = x*grid_size[0]+y
            a = index
            b = index+1
            c = index+grid_size[0]+1
            d = index+grid_size[0]
            tris.append((a, b, c))
            tris.append((a, c, d))
    return tris

def export_obj(vertices, tris, filename):
    file = open(filename, "w")
    for vertex in vertices:
      file.write("v " + str(vertex[0]) + " " + str(vertex[1]) + " " + str(vertex[2]) + "\n")
    for tri in tris:
      file.write("f " + str(tri[2]+1) + " " + str(tri[1]+1) + " " + str(tri[0]+1) + "\n")
    file.close()
    return

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

shape = (1024, 1024)  # Adjust the shape of the noise grid

x = np.linspace(0, shape[0], shape[0])
y = np.linspace(0, shape[1], shape[1])
X, Y = np.meshgrid(x, y)

noise = generate_perlin_noise_2d(shape, [256, 64, 16], [100, 10, 1])

plt.imshow(noise, cmap='gray', interpolation='nearest')
plt.colorbar()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, noise)

ax.set_zlim(-512, 512)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Height')

plt.show()