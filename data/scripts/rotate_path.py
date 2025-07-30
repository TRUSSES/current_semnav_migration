import pandas as pd
import os
from process_csv import path_dir

input_filename = os.path.join(path_dir(), 'robot2_success_region2_1_chrono.csv')
output_filename = os.path.join(path_dir(), 'robot2_success_region2_1_chrono_rot.csv')

df = pd.read_csv(input_filename)
x, y = df['x'], df['y']

# Define bounding box
min_x_bound = 0
min_y_bound = 0
max_x_bound = 600
max_y_bound = 400

# Translate line to be centered at the origin
center_x = (max_x_bound - min_x_bound) / 2
center_y = (max_y_bound - min_y_bound) / 2
x = x - center_x
y = y - center_y

# Rotate line 90 deg. CCW
x, y = -y, x

df['x'] = x
df['y'] = y
df = df.drop(columns=['orientation'])
df.to_csv(output_filename, index=False)