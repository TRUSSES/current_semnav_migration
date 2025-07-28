# Reactive Navigation with Semantic Feedback Using ROS2

This package has been adapted from [vvasilo/semnav](https://github.com/vvasilo/semnav). Please refer to the original repository for relevant publications, citations, and additional info.

This has been tested with Ubuntu 22.04 and ROS2 Humble, using a Turtlebot3 Waffle (differential drive) model in Gazebo 11.

>**Active branch**: `risk_map_integration` (for simulation and testing)

## Components
1. Navigation node (`navigation.cpp`): Computes and publishes `Twist` commands using semantic map inputs.
2. Fake map publisher (`fake_map_publisher.cpp`): Publishes semantic obstacles from polygon CSV files.
3. Fake LIDAR node (`fake_lidar_publisher.cpp`): Publishes dummy LIDAR data for planner input.
4. Launch file (`navigation_simulation.launch.py`): Runs planner, headless Gazebo, TurtleBot3, and Foxglove visualization bridge.

## Setup Instructions

### 1. ROS2 + Dependencies

- Install **ROS2 Humble**: [Create a ROS2 Workspace (Humble)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- Install dependencies listed in the original repo: Boost Geometry, CGAL, `earcut.hpp`, trippy, shapely, scipy, numpy, matplotlib, imagemagick
- Clone this repo into your `~/ros2_ws/src` workspace.
- Copy custom messsage definitions: `cp -r extras/object_pose_interface_msgs ~/ros2_ws/src/`
- Source environment: `source ~/ros2_ws/src/install/setup.bash`
- Build packages:
```
cd ~/ros2_ws/src
colcon build --packages-select object_pose_interface_msgs semnav
```
- Check installation: `ros2 pkg list | grep -e semnav -e object`

### 2. Gazebo + TurtleBot3 Setup

- Install Gazebo 11 (source or binaries).
- Install [TurtleBot3 for ROS2 Humble](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).
- Add model config: ```export TURTLEBOT3_MODEL=waffle```

### 3. Foxglove Setup
Run the following:
```
sudo apt update
sudo apt install ros-humble-foxglove-bridge ros-humble-foxglove-msgs
```

## Launch full simulation
The main launch file handles:
- Loading a polygonal obstacle map
- Starting Gazebo with the TurtleBot
- Launching the planner with parameters
- Launching a fake lidar sensor
- Starting Foxglove Bridge for visualization

**Note:** The `world` parameter used by `gzserver` should be updated to the absolute path of your repo.

Example usage:
```
ros2 launch semnav navigation_simulation.launch.py \
    x_pose:=2.0 \
    y_pose:=-6.0 \
    obstacle_file:="4x4rect.csv" \
    goal_x:=1.0 \
    goal_y:=2.0
```
Obstacle files are stored in `data/` with the following format:
- First row: X coordinates of each polygon.
- Second row: Y coordinates of each polygon.
- Polygons separated by `NaN`
- First polygon represents the workspace boundary, and is ignored when generating the map.

`(x_pose, y_pose)` is the spawn point of the robot in Gazebo.
 
## Visualizing in Foxglove
1. Start the launch file--it includes `foxglove_bridge`.
2. Open the [Foxglove Web app](https://app.foxglove.dev) or the [Foxglove Studio desktop](https://foxglove.dev/download).
3. Connect to the bridge via WebSocket.
``` WebSocket URL
ws://localhost:8765
```

Foxglove will automatically detect the `/geojson_map` topic, which contains the trajectory line and the obstacle polygons, and visualize them in a Map panel in realtime.
1. Open the Map panel.
2. Select `General > Tile Layer > Custom` to visualize with an empty background.

## Generating vector field plots (debugging)
Each vector points from the robot's initial pose to final pose at each grid point, given a fixed initial robot orientation, by following the planner's twist command for some time step *t*.

### Parameters
- `goal_x`, `goal_y`: coordinates of goal point (plotted in red).
- `min_x`, `max_x`, `min_y`, `max_y`: range of grid points to plot.
- `grid_n`: number of points to plot per row and per column.
- `target_x`, `target_y`: point for which local freespace (yellow), local linear goal (magenta star), and local angular goal (cyan star) will be plotted for debugging purposes.

> The local linear goal is the projection of the global goal onto the linear local freespace, which is the line segment from the robot's position, along its orientation vector, to the local freespace boundary. It is used to compute linear velocity.
>
> The local angular goal is the projection of the global onto the local freespace polygon. It is used to compute angular velocity.

To plot grid of vectors only:
```
ros2 run semnav vector_field_plot \
-- [goal x] [goal y] [min x] [max x] [min y] [max y] [grid n]
```

To plot single vector and planner parameters (LF, LGL, LGA) for target point:
```
ros2 run semnav vector_field_plot \
-- [goal x] [goal y] [target x] [target y]
```

## Generating path CSVs (mapping paper)
### 1. Record the path
- Launch `navigation_simulation.launch.py`.
- This will generate a `trajectory.csv` file in the **share** directory.
- Format of each line in `trajectory.csv`:
> index,x,y,orientation
### 2. Configure the scaling script (optional)
- Open `/data/scripts/scale_path_csv.py`.
- Update:
    - The input filename (default is `trajectory.csv`).
    - The output filename.
    - The **share** directory path (if different).
    - The scale factor *k* (default is 2.0).
### 3. Scale the path
- Run `scale_path_csv.py` to scale all *x* and *y* coordinates in the trajectory by *k*.
- The output will be written in the same CSV format.
- By default, the input and output files are assumed to be in the **share** directory.
- Tip: After scaling, you can move the output file to the data directory of the main repo for safekeeping.
### 4. Visualize the trajectory (optional)
- Use the `/data/scripts/plot_trajectory.py` script.
- Flags:
    - `--title`: plot title
    - `--obstacle_file`: CSV with obstacle data (assumed to be in data directory)
    - `--goal_x`, `--goal_y`: coordinates of goal
    - `--pattern`: CSV with trajectory data
    - `--in_share_dir`: if the trajectory CSV is in the **share** directory
    - `--in_data_dir`: if the trajectory CSV is in the **data** directory of the main repo.
- Assumes:
    - Start point of path is the first point in the CSV
    - If `--goal_x` and `--goal_y` arguments are passed: `(goal_x, goal_y)` is the goal point.
    - If `--goal_x` or `--goal_y` arguments are not passed: the last point in the CSV is the goal.

## Obstacle data pre-processing
### 1. Convert velocity map to obstacle CSV
- TODO: Script for converting a velocity map into obstacle CSV (outside the ROS2 node).
### 2. Scale the obstacle data (optional)
- Open `/data/scripts/scale_obstacle_csv.py`. 
- Update:
    - Scaling factors *kx* and *ky*.
    - Input and output filenames.
- This script:
    - Assumes that the obstacle CSV is in the **data** directory of the main repo.
    - Writes the scaled version to the same directory.

### Debugging Tips
- Use `ros2 topic list` to confirm that semnav `/reactive_planner/...` topics are running.
- Rebuild after code changes, source correctly in every terminal.
- If Gazebo isn't refreshing: `killall -9 gzserver`.
