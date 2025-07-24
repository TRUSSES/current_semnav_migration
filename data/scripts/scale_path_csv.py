# Generate new CSV with x, y points scaled up by k
import csv
import pandas as pd

# Assume path is in share directory; store output in same directory.
share_directory = '/home/neha/ros2_ws/install/semnav/share/semnav'
data_directory = share_directory + '/data'

input_filename = data_directory + '/' + 'trajectory.csv'
output_filename = data_directory + '/' + 'robot2_success.csv'
k = 2.0  # Scale factor

df = pd.read_csv(input_filename)

df["x"] = df["x"] * 2
df["y"] = df["y"] * 2

df.to_csv(output_filename, index=False)