import matplotlib.pyplot as plt
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import sys

if len(sys.argv) != 3:
    print("need plot title and map file args")

plot_title = sys.argv[1]
map_csv = sys.argv[2]

fig, ax = plt.subplots()

share_directory = '/home/neha/ros2_ws/src/install/semnav/share/semnav'
data_directory = share_directory + '/data'

# Obstacle map
map_file = data_directory + '/' + map_csv
df = pd.read_csv(map_file, header=None)
x_row = df.iloc[0].values
y_row = df.iloc[1].values

# New polygons start/end on first row value, NaN values, and last row value
split_indices = np.where(np.isnan(x_row))[0] 
split_indices = np.concatenate(([-1], split_indices, [len(x_row)]))

polygons = []
for i in range(len(split_indices) - 1):
    start = split_indices[i] + 0
    end = split_indices[i + 1]
    x_coords = x_row[start:end]
    y_coords = y_row[start:end]
    vertices = np.column_stack((x_coords, y_coords))
    polygons.append(Polygon(vertices, closed=True))
patches = PatchCollection(polygons, facecolor='lightblue')
ax.add_collection(patches)

# Trajectories
for filename in os.listdir(data_directory):
    if 'trajectory' in filename and 'csv' in filename:
        filename = os.path.join(data_directory, filename)
        df = pd.read_csv(filename)

        x = df['x'].to_numpy()
        y = df['y'].to_numpy()
        linear_x = df['linear_x'].to_numpy()
        linear_y = np.zeros_like(linear_x) # assume all zero

        # approximate angle as sum of all past angles (this might not work)
        theta = np.cumsum(df['angular_z'].fillna(0).to_numpy())

        # Rotate
        x_rot = linear_x * np.cos(theta)
        y_rot = linear_x * np.sin(theta)

        # trajectory
        ax.plot(x, y)

        # twist cmds at every nth point
        n = 10
        ax.quiver(
        x[::n], y_vals[::n],
        x_rot[::n], y_rot[::n],
        angles='xy', scale_units='xy', scale=1.0, color='red', width=0.0025
    )


plt.grid(True)
plt.gca().set_aspect('equal') # Set aspect ratio of x and y axes
plt.title(''.join([plot_title]))
plt.axis('equal')
plt.show()