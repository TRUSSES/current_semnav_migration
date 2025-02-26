import matplotlib.pyplot as plt
import pandas as pd
from ament_index_python.packages import get_package_share_directory

share_directory = '/home/neha/ros2_ws/src/install/semnav/share/semnav'
file_name = share_directory + '/data/trajectory.csv'
cols = ["x", "y"]
df = pd.read_csv(file_name, usecols=cols)

plt.plot(df.x, df.y)
plt.show()