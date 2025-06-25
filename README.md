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
Run:
```
sudo apt upgrade
sudo apt install ros-humble-foxglove-bridge ros-humble-foxglove-msgs
```

## Launch full simulation
The main launch file handles:
- Loading a polygonal obstacle map
- Starting Gazebo with the TurtleBot
- Launching the planner with parameters
- Launching a fake lidar sensor
- Starting Foxglove Bridge for visualization

Example usage:
```
ros2 launch semnav navigation_simulation.launch.py \
    x_pose:=2.0 \
    y_pose:=-6.0 \
    obstacle_file:="4x4rect.csv"
```
Obstacle files are stored in `data/` with the following format:
- First row: X coordinates of each polygon.
- Second row: Y coordinates of each polygon.
- Polygons separated by `NaN`
- First polygon represents the workspace boundary, and is ignored when generating the map.

 
## Visualizing in Foxglove
1. Start the launch file--it includes `foxglove_bridge`.
2. Open the [Foxglove Web app](https://app.foxglove.dev) or the [Foxglove Studio desktop](https://foxglove.dev/download).
3. Connect to the bridge via WebSocket.
``` WebSocket URL
ws://localhost:8765
```

> Foxglove will automatically detect the `/geojson_map` topic, which contains the trajectory line and the obstacle polygons, and visualize them in a Map panel in realtime.

### Debugging Tips
- Use `ros2 topic list` to confirm that semnav `/reactive_planner/...` topics are running.
- Rebuild after code changes, source correctly in every terminal.
- If Gazebo isn't refreshing: `killall -9 gzserver`.
