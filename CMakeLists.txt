cmake_minimum_required(VERSION 3.5)
project(semnav)

## Compile as C++14, supported in ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
find_package(object_pose_interface_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED) # For simulation

# Find other libraries
find_package(Boost REQUIRED COMPONENTS system)
find_package(CGAL REQUIRED COMPONENTS Core)

###################################
## catkin specific configuration ##
###################################
# catkin_package() not required in ROS 2

###########
## Build ##
###########
set(MAIN_DIR ${CMAKE_SOURCE_DIR})

## Specify additional locations of header files
ament_export_include_directories(include)
include_directories(
  ${MAIN_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${object_pose_interface_msgs_INCLUDE_DIRS}
  ${example_interfaces_INCLUDE_DIRS}
  ${foxglove_msgs_INCLUDE_DIRS}
)

## Declare C++ libraries
add_library(reactive_planner
  ${MAIN_DIR}/src/libraries/polygeom_lib.cpp
  ${MAIN_DIR}/src/libraries/reactive_planner_lib.cpp
)

add_library(generate_map
  ${MAIN_DIR}/data/scripts/generate_map.cpp
)

target_include_directories(reactive_planner PRIVATE
  ${MAIN_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${object_pose_interface_msgs_INCLUDE_DIRS}
  ${example_interfaces_INCLUDE_DIRS}
  ${foxglove_msgs_INCLUDE_DIRS}
)

target_link_libraries(reactive_planner
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${message_filters_LIBRARIES}
  ${object_pose_interface_msgs_LIBRARIES}
  ${example_interfaces_LIBRARIES}
  ${foxglove_msgs_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core
)

# Add executables
add_executable(navigation 
  ${MAIN_DIR}/src/navigation.cpp
)

add_executable(fake_map_publisher 
  ${MAIN_DIR}/src/fake_map_publisher.cpp
)

add_executable(fake_lidar_publisher 
  ${MAIN_DIR}/src/fake_lidar_publisher.cpp
)

add_executable(fake_odometry_publisher 
  ${MAIN_DIR}/src/fake_odometry_publisher.cpp
)

add_executable(velocity_visualizer
  ${MAIN_DIR}/src/velocity_visualization_node.cpp
)

# ament_package()
add_executable(turtlebot3_map_publisher 
  ${MAIN_DIR}/src/turtlebot3_map_publisher.cpp
)

# Link
target_link_libraries(navigation
  reactive_planner
)

target_link_libraries(fake_map_publisher
  reactive_planner
  generate_map
)

target_link_libraries(fake_lidar_publisher
  reactive_planner
)

target_link_libraries(fake_odometry_publisher
  reactive_planner
)

target_link_libraries(turtlebot3_map_publisher
  reactive_planner
)

target_link_libraries(velocity_visualizer
  reactive_planner
)


## Mark executables and/or libraries for installation
install(TARGETS
  reactive_planner
  generate_map
  navigation
  fake_map_publisher
  velocity_visualizer
  fake_lidar_publisher
  fake_odometry_publisher
  turtlebot3_map_publisher
  velocity_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

## Install Python executables
install(PROGRAMS
  src/tracking/trajectory_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

## Make "data" available
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})


## Declare Silence Boost Errors
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

#Silence Deprecation Warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -DBOOST_DISABLE_DEPRECATED_HEADERS")
add_definitions(-w)

ament_package()
