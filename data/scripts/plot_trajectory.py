import matplotlib.pyplot as plt
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from matplotlib.patches import Polygon

# Add rectangle obstacle (hard code for now)
vertices = np.array([[1, -2], [1, -6], [-3, -6], [-3, -2]])
obstacle = Polygon(vertices, facecolor='skyblue')

fig, ax = plt.subplots()
ax.add_patch(obstacle)

share_directory = '/home/neha/ros2_ws/src/install/semnav/share/semnav'
data_directory = share_directory + '/data'
for filename in os.listdir(data_directory):
    if 'trajectoryA' in filename and 'csv' in filename:
        filename = os.path.join(data_directory, filename)
        cols = ["x", "y"]
        df = pd.read_csv(filename, usecols=cols)
        plt.plot(df.x, df.y)

plt.grid(True)
plt.xticks(np.arange(-10, 10))
plt.yticks(np.arange(-10, 10))
plt.gca().set_aspect('equal') # Set aspect ratio of x and y axes
plt.title("Goal: (3, -4)")
plt.show()