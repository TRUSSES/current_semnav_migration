# Generate new CSV with x, y points scaled up by k
import csv
import pandas as pd
from process_csv import path_dir

# Assume path is in /data/paths and store in the same directory
input_filename = os.path.join(path_dir(), 'robot2_success_region2_1_chrono.csv')
output_filename = os.path.join(path_dir(), 'robot2_success_region2_1_chrono_rot.csv')
k = 4.0  # Scale factor

df = pd.read_csv(input_filename)

df["x"] = df["x"] * k
df["y"] = df["y"] * k

df.to_csv(output_filename, index=False)