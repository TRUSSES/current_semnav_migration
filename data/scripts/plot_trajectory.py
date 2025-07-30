import matplotlib.pyplot as plt
import pandas as pd
import os
import argparse
import numpy as np
import fnmatch
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import sys
from process_csv import obstacle_map_dir, path_dir

# Argument parsing
parser = argparse.ArgumentParser(description="Plot trajectories with obstacle map.")
parser.add_argument("--title", type=str, default="Trajectory plot", help="Plot title")
parser.add_argument("--obstacle_file", type=str, required=True, help="CSV file with obstacle data")
parser.add_argument("--goal_x", type=float, required=False, help="Goal X coordinate")
parser.add_argument("--goal_y", type=float, required=False, help="Goal Y coordinate")
parser.add_argument("--path_file", type=str, default="trajectory*.csv",
    help="Pattern to match trajectory files (e.g., 'trajectory*.csv')")
parser.add_argument("--in_share_dir",
    action="store_true",
    help="Enable if path CSV is in share directory"
)

args = parser.parse_args()

plot_title = args.title
map_csv = args.obstacle_file
goal_x = args.goal_x
goal_y = args.goal_y

fig, ax = plt.subplots()

# Plot obstacle map
map_file = os.path.join(obstacle_map_dir(), map_csv)
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

# Find trajectory file.
data_directory = None

if (args.in_share_dir):
    share_directory = '/home/neha/ros2_ws/install/semnav/share/semnav'
    data_directory = os.path.join(share_directory, 'data')
else:
    data_directory = path_dir()

print('data dir: ', data_directory)

# Plot trajectories.
x, y = None, None

for filename in os.listdir(data_directory):
    if fnmatch.fnmatch(filename, args.path_file):
        filename = os.path.join(data_directory, filename)
        df = pd.read_csv(filename)

        x = df['x'].to_numpy()
        y = df['y'].to_numpy()

        # trajectory
        ax.plot(x, y)

        # plot start point
        ax.plot(x[0], y[0], 'o', color='red', label=f"{round(x[0])}, {round(y[0])})")

# Plot goal point
if args.goal_x is None and args.goal_y is None:
    ax.plot(x[-1], y[-1], 'o', label=f"({round(x[-1])}, {round(y[-1])})")
else:
    ax.plot(goal_x, goal_y, 'o', label=f"({round(goal_x)}, {round(goal_y)})")

plt.grid(True)
plt.gca().set_aspect('equal') # Set aspect ratio of x and y axes
plt.title(''.join([plot_title]))
plt.axis('equal')
plt.xlabel('x-position (cm)')
plt.ylabel('y-position (cm)')
plt.legend(bbox_to_anchor=(1, 0.5))
plt.show()