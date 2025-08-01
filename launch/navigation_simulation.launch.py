import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


""" Launch file functions
    - Run navigation node with parameters.
    - Run fake lidar node.
    - Spawn turtlebot in Gazebo with initial pose arguments.
"""

def generate_launch_description():
    # Gazebo requirements
    tb_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    semnav_world_dir = os.path.join(get_package_share_directory('semnav'), 'worlds')
 
    # Gazebo launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    goal_x = LaunchConfiguration('goal_x', default='1.0')
    goal_y = LaunchConfiguration('goal_y', default='1.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # Secondary launch files for Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
		launch_arguments={
            'world': os.path.join(semnav_world_dir, 'slow_empty.sdf'),
            'verbose': 'true'
        }.items()  # world with different physics from default
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw
        }.items()
    )

    # Launch configuration variables for planner
    pub_twist_topic = LaunchConfiguration('pub_twist_topic')
    pub_behaviorID_topic = LaunchConfiguration('pub_behaviorID_topic')
    pub_behaviorMode_topic = LaunchConfiguration('pub_behaviorMode_topic')

    sub_robot_topic = LaunchConfiguration('sub_robot_topic')
    sub_laser_topic = LaunchConfiguration('sub_laser_topic')
    sub_semantic_topic = LaunchConfiguration('sub_semantic_topic')

    world_frame_id = LaunchConfiguration('world_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    laser_frame_id = LaunchConfiguration('laser_frame_id')

    # Fake map publisher requirements
    fake_map = LaunchConfiguration('fake_map', default='true')
    obstacle_file = LaunchConfiguration('obstacle_file', default='1x1rect.csv')

    return LaunchDescription([
        # Declare planner arguments
        DeclareLaunchArgument('pub_twist_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('pub_behaviorID_topic', default_value='/minitaur/command/behaviorId'),
        DeclareLaunchArgument('pub_behaviorMode_topic', default_value='/minitaur/command/behaviorMode'),

        DeclareLaunchArgument('sub_robot_topic', default_value='/odom'),
        DeclareLaunchArgument('sub_laser_topic', default_value='/scan'),
        #DeclareLaunchArgument('sub_laser_topic', default_value='/fake_lidar_scan'),
        DeclareLaunchArgument('sub_semantic_topic', default_value='/pose_tracking/semantic_map'),

        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('laser_frame_id', default_value='laser'),

        DeclareLaunchArgument('obstacle_file', default_value='1x1rect.csv'),

        # Gazebo and TurtleBot3 commands
        gzserver_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,

        # Launch the main planner node with the necessary parameters
        Node(
            package='semnav',
            executable='navigation',
            namespace='reactive_planner',
            name='navigation_node',
            output='screen',
            parameters=[{
                'pub_twist_topic': pub_twist_topic,
                'pub_behaviorID_topic': pub_behaviorID_topic,
                'pub_behaviorMode_topic': pub_behaviorMode_topic,
                'sub_laser_topic': sub_laser_topic,
                'sub_robot_topic': sub_robot_topic,
                'sub_semantic_topic': sub_semantic_topic,
                'world_frame_id': world_frame_id,
                'odom_frame_id': odom_frame_id,
                'laser_frame_id': laser_frame_id,

                # NUMERICAL PARAMETERS
                'RobotRadius': 0.3,
                'ObstacleDilation': 1.0,
                'WalkHeight': 0.5,

                'AllowableRange': 4.0,
                'CutoffRange': 0.15,

                'RFunctionExponent': 20.0,
                'Epsilon': 5.0,
                'VarEpsilon': 5.0,
                'Mu1': 0.8,
                'Mu2': 0.05,
                'WorkspaceMinX': -10.0,
                'WorkspaceMinY': -10.0,
                'WorkspaceMaxX': 200.0,
                'WorkspaceMaxY': 200.0,
                'SemanticMapUpdateRate': 10.0,

                'ForwardLinCmdLimit': 0.3,
                'BackwardLinCmdLimit': 0.0,
                'AngCmdLimit': 0.7,

                'LinearGain': 0.2,
                'AngularGain': 0.4,

                'Goal_x': goal_x,
                'Goal_y': goal_y,
                'Tolerance': 0.4,

                'LowpassCutoff': 4.0,
                'LowpassSampling': 25.0,
                'LowpassOrder': 6.0,
                'LowpassSamples': 10.0,

                'DebugFlag': False,
				'SimulationFlag': True, # Publish msgs for Foxglove visualization
            }]
        ),

        Node(
            package='semnav',  
            executable='fake_map_publisher',  
            namespace='reactive_planner',
            name='fake_map_publisher',
            output='screen',
            condition=IfCondition(fake_map),
            parameters=[{
                'pub_semantic_topic': '/pose_tracking/semantic_map',  
                'pub_transform_topic': '/pose_tracking/world_frame', 
                'obstacle_file': obstacle_file, 
                'x_pose': x_pose,
                'y_pose': y_pose
            }]
        ),

        # Launch node that publishes fake lidar data for the planner.
        Node(
            package='semnav',  
            executable='fake_lidar_publisher',  
            namespace='reactive_planner',
            name='fake_lidar_publisher',
            output='screen',
            parameters=[{
                'pub_lidar_topic': '/fake_lidar_scan',  
            }]
        ),
 
		# Launch trajectory recording node
        Node(
            package='semnav',
            executable='trajectory_recorder.py',
            name='trajectory_recorder',
            output='screen'
		), 

        # Launch Foxglove Bridge as a ROS2 node.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        )
    ])
