import numpy as np
import matplotlib.pyplot as plt
from process_csv import obstacle_csv_to_polygons, obstacle_csv_to_lists
import argparse

# Get input filename
parser = argparse.ArgumentParser(description="Plot obstacles.")
parser.add_argument("--obstacle_file", type=str, required=True, help="CSV file with obstacle map data")
args = parser.parse_args()

# Process obstacle CSV
input_filename = args.obstacle_file
polygons = obstacle_csv_to_polygons(input_filename)

# Plot polygons
plt.figure(figsize=(6,4))
for poly_x, poly_y in polygons[1:]:
    #plt.plot(np.array(poly_x), np.array(poly_y), marker='o')  # lines with points
    plt.fill(np.array(poly_x), np.array(poly_y), alpha=1.0, color='red')  # fill polygons lightly

plt.title('Polygon Visualization')
plt.xlabel('X coordinates')
plt.ylabel('Y coordinates')
plt.axis('equal')
plt.grid(True)
plt.show()