# Generate new CSV with x, y points scaled up by k
import csv
import pandas as pd
import numpy as np

# Assume path is in data directory
input_filename = '../' + 'robot1_success_region3_1.csv'
output_filename = '../' + 'robot1_success_region3_1_trimmed.csv'

df = pd.read_csv(input_filename)

# Keep valid indices; include the first element
dist_threshold = 5
kept_indices = [0]
last_kept_x = df.loc[0, 'x']
last_kept_y = df.loc[0, 'y']

for i in range(1, len(df)):
    x = df.loc[i, 'x']
    y = df.loc[i, 'y']
    dist = np.sqrt((x - last_kept_x)**2 + (y - last_kept_y)**2)
    # Keep point if significant distance change
    if dist >= dist_threshold:
        kept_indices.append(i)
        last_kept_x = x
        last_kept_y = y

new_df = df.loc[kept_indices]
print(f'old row count: {len(df)}')
print(f'new row count: {len(new_df)}')

new_df.to_csv(output_filename, index=False)