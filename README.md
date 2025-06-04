# Reactive Navigation with Semantic Feedback Using ROS2

This package has been adapted from [vvasilo/semnav](https://github.com/vvasilo/semnav). Please refer to the original repository for relevant publications, citations, and additional info.

This has been tested with Ubuntu 22.04 and ROS2 Humble, using a Turtlebot3 Waffle Pi (differential drive) model in Gazebo 11.

>**Branch in use**: `neha` (for simulation and testing)

## Components
1. Navigation node (`navigation.cpp`): Uses semantic map as input, runs the planning algorithm, and publishes Twist commands.
2. Fake map publisher for simulation (`map_debug.cpp`).

## Setup Instructions

### 1. ROS2 + Dependencies

- Install **ROS2 Humble**: [Create a ROS2 Workspace (Humble)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- Install dependencies from the original repo: Boost Geometry, CGAL, earcut.hpp, trippy, shapely, scipy, numpy, matplotlib, imagemagick
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

- Install Gazebo 11, possibly from source since this is not the default version.
- Install [TurtleBot3 (ROS2 Humble)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- Add environment variables (to each terminal):
```
export TURTLEBOT3_MODEL=waffle_pi
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
```
Some warnings/errors are fine, as long as TURTLEBOT3_MODEL is set correctly.

## Simulation Instructions

### 1. Polygon Map Setup
- Store your obstacle CSV file in `~/ros2_ws/src/current_semnav_migration/data/`
    - First row: X coordinates
    - Second row: Y coordinates
    - Polygons separated by `NaN`
    - First polygon is ignored (used as boundary)
- Add CSV file path in `map.debug.cpp` constructor (TODO: pass as parameter instead), and rebuild package.
- Functions used in `map.debug.cpp`:
    - `get_polygons()`: Parses polygons from CSV
    - `generate_sdf()`: Generates the SDF model, where obstacles are visual only (no collision properties)
 
### 2. Launch semnav
```
ros2 launch semnav sim_navigation_turtlebot3.launch.py
```
This will:
- Read and parse polygon CSV.
- Publish semantic map of polygons.
- Publish fake LIDAR data.
- Set parameters for the planning algorithm and run the navigation node.

### 3. Launch TurtleBot3 Simulation
```
ros2 launch semnav turtlebot3_polygon_world.launch.py \
    x_pose:=[start_x] y_pose:=[start_y]
```

### 4. Record Trajectories
```
ros2 runs semnav trajectory_recorder.py \
    --ros-args -p output_file:=[trajectory_file_name].csv
```
- Output saved in share directory: `~/ros2_ws/src/install/semnav/share/semnav/data/`
- Records real-time poses.
- Stop with `CTRL+C`

### 5. Plot Trajectories
- Edit `data/scripts/plot_trajectory.py` to hardcode the correct share directory path (for now) and pick which trajectory files to use. Default is to select any files named `trajectory*.csv`.
- Run:
```
python3 plot_trajectory.py "Plot Title" "map_file.csv"
```
where `map_file.csv` is the name of the file in `/data` that contains your obstacles.

### Debugging Tips
- Use `ros2 topic list` to confirm semnav topics are running.
- Rebuild after code changes, source correctly in every terminal.
- If Gazebo isn't refreshing: `killall -9 gzserver gzclient`.
- Ensure you're using the correct share directory for CSV I/O.
