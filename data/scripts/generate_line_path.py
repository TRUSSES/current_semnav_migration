# Creates CSV with line path from start to goal point
import pandas as pd
import numpy as np
import math
import argparse

# Argument parsing
parser = argparse.ArgumentParser(description="Generate line path between 2 points.")
parser.add_argument("--start_x", type=float, required=True, help="Start X coordinate")
parser.add_argument("--start_y", type=float, required=True, help="Start Y coordinate")
parser.add_argument("--goal_x", type=float, required=True, help="Goal X coordinate")
parser.add_argument("--goal_y", type=float, required=True, help="Goal Y coordinate")
parser.add_argument("--output_file", type=str, required=True, help="CSV file to write path data")
parser.add_argument("--num_points", type=int, default=10, help="Number of points in path")

args = parser.parse_args()

start_x = args.start_x
start_y = args.start_y
goal_x = args.goal_x
goal_y = args.goal_y
out_filename = args.output_file
num_points = args.num_points

out_file = '../' + out_filename

# Coordinates on path
x = np.linspace(start_x, goal_x, num_points)
y = np.linspace(start_y, goal_y, num_points)

# Constant orientation
theta = math.atan2(goal_x - start_x, goal_y - start_y)

# Create dataframe and store in CSV
df = pd.DataFrame({
    'index': range(num_points),
    'x': x,
    'y': y,
    'orientation': [theta] * num_points
})

df.to_csv(out_file, index=False)