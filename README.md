# Reactive Navigation with Semantic Feedback Using ROS2

This package has been adapted from [vvasilo/semnav](https://github.com/vvasilo/semnav). Please refer to the original repository for relevant publications, citations, and additional info.

This has been tested with Ubuntu 22.04 and ROS2 Humble, using a Turtlebot3 Waffle (differential drive) model in Gazebo 11.

>**Active branch**: `main` (includes simulation and testing infrastructure)

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
The main launch file:
- Processes a static polygonal obstacle map and passes it to a fake map publisher node.
- Starts Gazebo with the TurtleBot using initial pose parameters.
- Launches the planner with changeable algorithm parameters.
- Launches a fake lidar publisher node.
- Starts Foxglove Bridge for real-time visualization.

Robot parameters, planner parameters, and input/output filenames can be configured in `/config/launch_args.yaml`. Rebuild and source after changing the YAML file.

Example usage:
```
ros2 launch semnav navigation_simulation.launch.py \
    x_pose:=2.0 \
    y_pose:=-6.0 \
    obstacle_file:="4x4rect.csv" \
    goal_x:=1.0 \
    goal_y:=2.0
```

**Important:**
- Reference `/data/README.md` for all file formats and the directories in which they should be stored.
- Absolute paths to data directories can be changed in `/data/scripts/process_csv.py`.

`(x_pose, y_pose)` is the spawn point of the robot in Gazebo.
 
## Visualizing in Foxglove
1. Set `SimulationFlag` to `true` in `/config/launch_args.yaml`. 
2. Start the launch file--it includes `foxglove_bridge`.
3. Open the [Foxglove Web app](https://app.foxglove.dev) or the [Foxglove Studio desktop](https://foxglove.dev/download).
4. Connect to the bridge via WebSocket.
``` WebSocket URL
ws://localhost:8765
```

Foxglove will automatically detect the `/geojson_map` topic, which contains the trajectory line and the obstacle polygons, and visualize them in a Map panel in realtime.
1. Open the Map panel.
2. Select `General > Tile Layer > Custom` to visualize with an empty background.

## Recording path CSVs (for visualization and mapping paper)
### 1. Record the path
- Launch `navigation_simulation.launch.py`.
- This will generate a `trajectory.csv` file in `/data/paths`.
- See `/data/README.md` for output CSV format details.

### 2. Scale or rotate the path data (optional)
- Open `/data/scripts/scale_path.py` or `/data/scripts/rotate_path.py`.
- Update:
    - The input filename.
    - The output filename.
    - The scale factor *k*.

### 3. Visualize the trajectory (optional)
- Use the `/data/scripts/plot_trajectory.py` script.
- Flags:
    - `--title`: plot title
    - `--obstacle_file`: CSV with obstacle data, assumed to be in `/data/obstacle_maps`.
    - `--path_file`: CSV with trajectory data, assumed to be in `/data/paths`.
    - `--goal_x`, `--goal_y`: coordinates of goal
    - `--in_share_dir`: (optional) True if the trajectory CSV is in the **share** directory.
- Assumptions:
    - Start point of path is the first point in the CSV
    - If goal point is specified (`--goal_x`, `--goal_y`), then `(goal_x, goal_y)` is the goal point.
    - If no goal point specified, then the last point in the CSV is the goal.

## Obstacle data processing
### 1. Convert risk map to obstacle CSV
- In `/config/launch_args.yaml`, change `risk_in_file` and `obstacle_out_file` to the desired filenames.
- Run `/data/scripts/risk_to_obstacle_map.py` to visualize the extracted obstacles over the risk map, and generate an obstacle file in `/data/obstacle_maps`.

### 2. Transform the obstacle data (optional)
- Open `/data/scripts/transform_obstacle_csv.py`. 
- Update:
    - Input and output filenames.
    - Modify `main()` to execute the desired transformations.
    - Scaling factors *kx* and *ky* (optional).
- Run the script to generate transformed obstacle data and write it to a CSV.

### Debugging Tips
- Use `ros2 topic list` to confirm that semnav `/reactive_planner/...` topics are running.
- Rebuild after code changes, source correctly in every terminal.
- If Gazebo isn't refreshing: `killall -9 gzserver`.
