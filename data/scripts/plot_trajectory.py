import matplotlib.pyplot as plt
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import sys

if len(sys.argv) != 5:
    print("need plot title and map file args")

"""
shell cmd: python3 plot_trajectory.py [title] [csv file with obstacle data] [goal_x] [goal_y]
"""
plot_title = sys.argv[1]
map_csv = sys.argv[2]
goal_x = float(sys.argv[3])
goal_y = float(sys.argv[4])

fig, ax = plt.subplots()

share_directory = '/home/neha/ros2_ws/src/install/semnav/share/semnav'
data_directory = share_directory + '/data'

# Plot obstacle map
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

""" Plot trajectories.
Hardcode input files with trajectory data for now. Should be in [share dir]/data/
2x2trajectory...csv -> 1x1 obstacle lol
1x1trajectory.csv -> 2x2 obstacle
"""
for filename in os.listdir(data_directory):
    if 'originaltrajectory' in filename and 'csv' in filename:
        filename = os.path.join(data_directory, filename)
        df = pd.read_csv(filename)

        x = df['x'].to_numpy()
        y = df['y'].to_numpy()
        linear_x = df['linear_x'].to_numpy()
        linear_y = np.zeros_like(linear_x) # assume all zero

        # approximate angle as sum of all past angles (this might not work)
        theta = df['angular_z'].to_numpy()

        # Rotate
        x_rot = linear_x * np.cos(theta)
        y_rot = linear_x * np.sin(theta)

        # trajectory
        ax.plot(x, y)

        # twist cmds at every nth point
        """ Plot twist cmd vectors
        n = 20
        ax.quiver(
            x[::n], y[::n],
            x_rot[::n], y_rot[::n],
            angles='xy', scale_units='xy', scale=0.8, color='red', width=0.0025
        )
        """

# Plot goal point
ax.plot(goal_x, goal_y, 'o')

plt.grid(True)
plt.gca().set_aspect('equal') # Set aspect ratio of x and y axes
plt.title(''.join([plot_title]))
plt.axis('equal')
plt.show()