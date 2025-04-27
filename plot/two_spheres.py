import numpy as np
import matplotlib.pyplot as plt

def plot_two_offset_spheres(offset=(3, 1, 1)):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_box_aspect([1, 1, 1])

    r = 1
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    u, v = np.meshgrid(u, v)
    u = u.flatten()
    v = v.flatten()

    x = r * np.cos(u) * np.sin(v)
    y = r * np.sin(u) * np.sin(v)
    z = r * np.cos(v)

    ax.scatter(x, y, z, color='blue', s=1)
    ax.scatter(0, 0, 0, color='blue', s=50, label='Center 1')

    x2 = x + offset[0]
    y2 = y + offset[1]
    z2 = z + offset[2]
    ax.scatter(x2, y2, z2, color='red', s=1)
    ax.scatter(offset[0], offset[1], offset[2], color='red', s=50, label='Center 2')

    ax.legend()
    plt.show()

plot_two_offset_spheres(offset=(0.7, 0.7, 0.7))
