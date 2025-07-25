# Generate new CSV with x, y points scaled up by k
import csv
import pandas as pd

# Assume path is in share directory; store output in same directory.
share_directory = '/home/neha/ros2_ws/install/semnav/share/semnav'
data_directory = share_directory + '/data'

input_filename = data_directory + '/' + 'trajectory.csv'
output_filename = data_directory + '/' + 'robot1_success_region3_1.csv'
k = 4.0  # Scale factor

df = pd.read_csv(input_filename)

df["x"] = df["x"] * k
df["y"] = df["y"] * k

df.to_csv(output_filename, index=False)