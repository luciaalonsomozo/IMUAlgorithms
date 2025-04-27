import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_sphere_and_ellipsoid():
    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_box_aspect([1, 1, 1])

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    r = 1
    x_sphere = r * np.outer(np.cos(u), np.sin(v))
    y_sphere = r * np.outer(np.sin(u), np.sin(v))
    z_sphere = r * np.outer(np.ones_like(u), np.cos(v))
    ax.scatter(x_sphere, y_sphere, z_sphere, color='red', s = 0.5, alpha=0.5)

    a, b, c = 0.9, 1, 0.6 
    x_ellipsoid = a * np.outer(np.cos(u), np.sin(v))
    y_ellipsoid = b * np.outer(np.sin(u), np.sin(v))
    z_ellipsoid = c * np.outer(np.ones_like(u), np.cos(v))
    ax.scatter(x_ellipsoid, y_ellipsoid, z_ellipsoid, color='blue', s= 0.5, alpha=0.5)

    plt.show()

plot_sphere_and_ellipsoid()
