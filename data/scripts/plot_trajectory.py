import matplotlib.pyplot as plt
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from matplotlib.patches import Polygon

fig, ax = plt.subplots()

# Add rectangle obstacle (hard code for now)
""" file_multirect
vertices = np.array([
    [[1, -2], [1, -6], [-3, -6], [-3, -2]],
    [[1, 3], [1, 7], [-3, 7], [-3, 3]]
])
obstacle1 = Polygon(np.array(vertices[0]), facecolor='skyblue')
obstacle2 = Polygon(np.array(vertices[1]), facecolor='skyblue')

ax.add_patch(obstacle1)
ax.add_patch(obstacle2)
"""

# file_rect
vertices = np.array([[1, -2], [1, -6], [-3, -6], [-3, -2]])
obstacle = Polygon(vertices, facecolor='skyblue')

ax.add_patch(obstacle)

share_directory = '/home/neha/ros2_ws/src/install/semnav/share/semnav'
data_directory = share_directory + '/data'
for filename in os.listdir(data_directory):
    if 'trajectory' in filename and 'csv' in filename:
        filename = os.path.join(data_directory, filename)
        cols = ["x", "y"]
        df = pd.read_csv(filename, usecols=cols)
        print(df.x)
        plt.plot(df.x.to_numpy(), df.y.to_numpy())

plt.grid(True)
plt.xticks(np.arange(-10, 10))
plt.yticks(np.arange(-10, 10))
plt.gca().set_aspect('equal') # Set aspect ratio of x and y axes
plt.title("Goal: (3, -4)")
plt.show()