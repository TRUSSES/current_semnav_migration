import matplotlib.pyplot as plt
import pandas as pd
import os
import argparse
import numpy as np
import fnmatch
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import sys

# Argument parsing
parser = argparse.ArgumentParser(description="Plot trajectories with obstacle map.")
parser.add_argument("--title", type=str, default="Trajectory plot", help="Plot title")
parser.add_argument("--obstacle_file", type=str, required=True, help="CSV file with obstacle data")
parser.add_argument("--goal_x", type=float, required=True, help="Goal X coordinate")
parser.add_argument("--goal_y", type=float, required=True, help="Goal Y coordinate")
parser.add_argument("--pattern", type=str, default="trajectory*.csv",
    help="Pattern to match trajectory files (e.g., 'trajectory*.csv')")

args = parser.parse_args()

plot_title = args.title
map_csv = args.obstacle_file
goal_x = args.goal_x
goal_y = args.goal_y

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
2x2trajectory...csv -> 1x1 obstacle (my bad))
1x1trajectory.csv -> 2x2 obstacle
"""
for filename in os.listdir(data_directory):
    if fnmatch.fnmatch(filename, args.pattern):
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

        # Plot twist cmds at every nth point
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