# Data Formats

This directory contains all data formats used by the planner, and descriptions of Python tools to manipulate data.

## Folder structure

- `obstacles/` — CSVs with x, y points of polygonal obstacles. NaN separates polygons.
- `risk_maps/` — CSVs with risk value at each (x, y) point.
- `scripts/` — Python tools for transforming or generating map data.
- `paths/` — Cleaned and generated trajectory data for simulation runs.

## CSV Format: *obstacle_maps/*

Each file contains two rows:
1. First row: x-coordinates
2. Second row: y-coordinates  
`NaN` separates individual polygons.
The first value in each row must be NaN.

Example:
``` mass_5.0_1.5x1m.csv
x: NaN,0.0,150.0,150.0,0.0,NaN,0.3,18.2,13.0,NaN, ...
y: NaN,0.0,0.0,100.0,100.0,NaN,115.0,120.0,180.1,NaN, ...
```

## CSV Format: *risk_maps/*

1. Each file has the following header: `x,y,k`.
2. Each row specifies the x-coordinate, y-coordinate, and risk value of a point.

Example:
```terrain_map.csv
x,y,k
0,0,0.0
1,0,19.0
2,0,36.0
3,0,51.0
...
```

## CSV Format: *paths/*

1. Each file has the following header: `index,x,y,orientation` or `index,x,y`.
2. Each row specifies a robot pose along the trajectory.

**Notes:**
- These files are generated automatically when `navigation_simulation.launch.py` is run.
- Scripts that transform paths will store the result in `paths/` by default.

## Descriptions: *scripts/*

### Obstacle data tools
- `disjoint.py`: Helper functions used for obstacle extraction from a risk map.
- `generate_map.cpp`: Functions to extract obstacles from a CSV in vector format and write them to an SDF world file.
- `plot_obstacle.py`: Visualizes an obstacle map.
- `process_csv.py`: Functions to get paths to CSV directories, and convert obstacle map CSV data to formats that other scripts use.
- `risk_to_obstacle_map.py`: Extracts obstacles from a risk map, plots them on the colorized risk map, and stores the obstacle data as a CSV.
- `transform_obstacle_csv.py`: Functions to rotate, scale, and center obstacle data at the origin and generate a new map.

All input and output files are assumed to be stored in `obstacle_maps/` in the required format.

### Path data tools
- `generate_line_path.py`: Generates CSV of waypoints on a straight line from start point to goal point.
- `plot_trajectory.py`: Visualizes path pose data on top of an obstacle map.
- `rotate_path.py`: Generates CSV with path data rotated 90 deg. CCW.
- `scale_path.py`: Generates CSV with path data scaled by a constant *k*.
- `trim_path.py`: Processes path CSV and removes points that are too close together spatially.

All input and output files are assumed to be stored in `paths/` in the required format.