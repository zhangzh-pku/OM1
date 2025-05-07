import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from mpl_toolkits.mplot3d import Axes3D

# Generate noisy 3D points
num_points = 1000
x = np.random.uniform(-3, 10, num_points)
y = np.random.uniform(-3, 10, num_points)
z = np.sin(np.pi * x) * np.cos(np.pi * y) + np.random.normal(0, 10, num_points)

# Create a grid
xi = np.linspace(x.min(), x.max(), 100)
yi = np.linspace(y.min(), y.max(), 100)
X, Y = np.meshgrid(xi, yi)

# Interpolate the data onto the grid
Z = griddata((x, y), z, (X, Y), method='cubic')

# Create the figure and axes object
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
ax.plot_surface(X, Y, Z, cmap='viridis')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()