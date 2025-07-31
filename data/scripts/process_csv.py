# Converts obstacle CSV format to usable formats for Python/C++ scripts.
import pandas as pd
import numpy as np
import os

def obstacle_map_dir():
    return '/home/neha/ros2_ws/src/reactive_planner/data/obstacle_maps'

def path_dir():
    return '/home/neha/ros2_ws/src/reactive_planner/data/paths'

def obstacle_csv_to_lists(input_filename):
    input_filename = os.path.join(obstacle_map_dir(), input_filename)

    df = pd.read_csv(input_filename, header=None, dtype=float)
    df = df.astype(float)

    x = df.iloc[0]
    y = df.iloc[1]
    return x, y

"""
Returns list of tuples, where each tuple contains a list of x-coords
and a list of y-coords for each of the polygon's vertices.
"""
def obstacle_csv_to_polygons(input_filename):
    x, y = obstacle_csv_to_lists(input_filename)

    # indices that separate polygons
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


def make_unique_filename(filename):
    """
    If filename exists, append a number before the extension to make it unique.
    e.g., 'output.csv' -> 'output1.csv', 'output2.csv', etc.
    """
    if not os.path.exists(filename):
        return filename  # no conflict, return original
    
    base, ext = os.path.splitext(filename)
    counter = 1
    
    while True:
        new_filename = f"{base}{counter}{ext}"
        if not os.path.exists(new_filename):
            return new_filename
        counter += 1