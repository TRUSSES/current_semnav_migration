import numpy as np
import matplotlib.pyplot as plt

# Your data rows as lists (replace NaN with np.nan for processing)
x_coords = [0.0,0.0,150.0,150.0,np.nan,0.375,0.375,149.625,149.625,115.875,99.33,94.08,99.3525,76.125,62.625,40.875,21.87075,9.375,np.nan,0.375,0.375,8.625,22.875,46.125,52.125,74.625,84.375,104.76,136.875,149.625,149.625]
y_coords = [100.0,0.0,0.0,100.0,np.nan,95.89965,99.77,99.77,40.11,37.63,41.75,50.75,73.75,81.101,97.2736,91.41,95.25,89.3145,np.nan,0.230000000000004,76.312,85.086,77.6985,79.312,44.805,39.095,24.69,12.75,12.75,20.019999999999996,0.230000000000004]

# Convert lists to numpy arrays for easier handling
x = np.array(x_coords)
y = np.array(y_coords)

# Function to split coords on np.nan and return list of polygon parts
def split_polygons(x, y):
    isnan = np.isnan(x) | np.isnan(y)
    indices = np.where(isnan)[0]
    polygons = []
    start = 0
    for idx in indices:
        polygons.append((x[start:idx], y[start:idx]))
        start = idx + 1
    # last polygon
    polygons.append((x[start:], y[start:]))
    return polygons

polygons = split_polygons(x, y)

# Plot polygons
plt.figure(figsize=(6,4))
for poly_x, poly_y in polygons:
    plt.plot(poly_x, poly_y, marker='o')  # lines with points
    plt.fill(poly_x, poly_y, alpha=0.3)  # fill polygons lightly

plt.title('Polygons Visualization')
plt.xlabel('X coordinates')
plt.ylabel('Y coordinates')
plt.axis('equal')
plt.grid(True)
plt.show()