import pandas as pd
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import numpy as np
import math

from scipy.spatial import ConvexHull
import shapely as sp
from shapely.geometry import Polygon, Point, MultiPolygon
from shapely.ops import unary_union
from concave_hull import concave_hull

from disjoint import build_disjoint_sets # in data/scripts

def generate_test_data(output_filename):
    width, height = 20, 20
    data = []
    for y in range(height):
        for x in range(width):
            k = -((x - width / 2)**2) + 100
            data.append((x, y, k))
    df = pd.DataFrame(data, columns=['x', 'y', 'k'])
    df.to_csv(output_filename, index=False)

# load data into 2d array, assuming terrain map format is x,y,k
def df_to_grid(df):
    width = df['x'].max() + 1
    height = df['y'].max() + 1

    arr = np.zeros((height, width))

    for _, row in df.iterrows():
        arr[int(row['y'])][int(row['x'])] = row['k']
    
    return arr

"""
Obstacle extraction functions from mapping_package/utils
"""
def get_unsafe_points_from_map(Y, x_coords, y_coords, threshold):
    """
    Args:
        Y: flattened value (environment) map
        x_coords: x position of each value in Y
        y_coords: y position of each value in Y
        threshold: values below this are considered 'unsafe'
    
    Assume Y, x_coords, and y_coords have the same shape.
        
    Returns:
        position_set_filtered: Array of safe (x, y) locations
    """

    # Set of safe array indices
    Y = np.array(Y)
    unsafe_indices = np.where(Y < threshold)[0]

    # Set of safe real-world coordinates
    position_set_filtered = [(x_coords[i], y_coords[i]) for i in unsafe_indices]
   
    return position_set_filtered

def get_obstacles(Y, x_coords, y_coords, threshold):
    """
    Create obstacle polygons from filtered safe positions.
    
    Args:
        position_set_filtered: Array of safe positions
        
    Returns:
        polygon_list: List of obstacle polygons
    """

    position_set_filtered = get_unsafe_points_from_map(Y, x_coords, y_coords, threshold)

    # Build disjoint sets from the position set
    disjoint_sets = build_disjoint_sets(position_set_filtered, threshold=1)

    polygon_list = []
    
    # Create polygons from the disjoint sets
    for i, group in enumerate(disjoint_sets.subsets()):
        if len(group) > 2:
            points = [position_set_filtered[i] for i in group]
            points = np.array(points)
            # Check if points are collinear
            if np.all(points[:, 0] == points[0, 0]):
                print('points are collinear')
                continue
            print('points: ', points)
            
            # Create concave hull for larger groups
            if len(group) > 3:
                hull = concave_hull(points, concavity=1, length_threshold=0)

                # Add all polygons if alphashape created multiple from the cluster
                if isinstance(hull, MultiPolygon):
                    for poly in hull:
                        polygon_list.append(poly)
                    continue
                else:
                    poly = Polygon(hull)
            else:
                poly = Polygon(points) 
                
            polygon_list.append(poly)
    
    return polygon_list, disjoint_sets

# read terrain map and extract obstacles
input_filename = "../terrain_map_test.csv"
generate_test_data(input_filename)
df = pd.read_csv(input_filename)
threshold = 60.0

polygon_list, disjoint_sets = get_obstacles(
    df['k'], df['x'], df['y'], threshold
)
print('polygon list: ', polygon_list)

# terrain map to 2D array
data = df_to_grid(df)

# plot obstacles over heatmap
fig, ax = plt.subplots()
heatmap = ax.imshow(data, cmap='viridis')

for poly in polygon_list:
    x, y = poly.exterior.xy
    patch = patches.Polygon(np.column_stack((x, y)),
        facecolor='none',
        edgecolor='red',
        linewidth=2
    )
    ax.add_patch(patch)

plt.title("Stiffness map with extracted obstacles")
plt.colorbar(heatmap)
plt.show()

"""
Obstacle map CSV format:
row 0: x coords, with polygons separated by NaN.
row 1: y coords, with polygons separated by NaN.
First polygon is a bounding box (workspace).
"""

workspace_width, workspace_height = 100, 100
x_coords, y_coords = [], []

workspace_poly = Polygon([
    (0, 0),
    (workspace_width, 0),
    (workspace_width, workspace_height),
    (0, workspace_height)
])
polygon_list.insert(0, workspace_poly)

for poly in polygon_list:
    x, y = poly.exterior.xy
    x_coords.extend(x)
    y_coords.extend(y)
    x_coords.append("NaN")
    y_coords.append("NaN")

map_filename = '../test_semantic_map.csv'
map_df = pd.DataFrame([x_coords, y_coords])
map_df.to_csv(map_filename, index=False, header=False)