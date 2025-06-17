import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	# Launch configuration variables
	pub_twist_topic = LaunchConfiguration('pub_twist_topic')
	pub_behaviorID_topic = LaunchConfiguration('pub_behaviorID_topic')
	pub_behaviorMode_topic = LaunchConfiguration('pub_behaviorMode_topic')

	sub_robot_topic = LaunchConfiguration('sub_robot_topic')
	sub_laser_topic = LaunchConfiguration('sub_laser_topic')
	sub_semantic_topic = LaunchConfiguration('sub_semantic_topic')

	world_frame_id = LaunchConfiguration('world_frame_id')
	odom_frame_id = LaunchConfiguration('odom_frame_id')
	laser_frame_id = LaunchConfiguration('laser_frame_id')

	obstacle_file = LaunchConfiguration('obstacle_file')

	# Declare launch arguments
	return LaunchDescription([

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

		# Launch the main node with the necessary parameters
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
			 'ObstacleDilation': 0.3,
			 'WalkHeight': 0.5,

			 'AllowableRange': 4.0,
			 'CutoffRange': 0.15,

			 'RFunctionExponent': 20.0,
			 'Epsilon': 1.0,
			 'VarEpsilon': 1.0,
			 'Mu1': 0.8,
			 'Mu2': 0.05,
			 'SemanticMapUpdateRate': 8.0,

			 'ForwardLinCmdLimit': 0.3,
			 'BackwardLinCmdLimit': 0.0,
			 'AngCmdLimit': 0.7,

			 'LinearGain': 0.5,
			 'AngularGain': 2.0,

			 'Goal_x': 3.0,
			 'Goal_y': -4.0,
			 'Tolerance': 0.4,

			 'LowpassCutoff': 4.0,
			 'LowpassSampling': 25.0,
			 'LowpassOrder': 6.0,
			 'LowpassSamples': 10.0,

			 'DebugFlag': True,
		  }]
		),

		Node(
		  package='semnav',  
		  executable='map_debug',  
		  namespace='reactive_planner',
		  name='fake_slam_publisher',
		  output='screen',
		  parameters=[{
			 'pub_semantic_topic': '/pose_tracking/semantic_map',  
			 'pub_transform_topic': '/pose_tracking/world_frame', 
			 'obstacle_file': obstacle_file, 
		  }]
		),

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

		# Node(
		#	package='semnav',
		#	executable='fake_odometry_publisher',  
		#	namespace='reactive_planner',
		#	name='fake_odometry_publisher',
		#	output='screen',
		#	parameters=[{
		#		'pub_odom_topic': '/fake_odom',  
		#	}]
		# ),

		# Node(
		#	package='semnav',  
		#	executable='turtlebot3_map_publisher', 
		#	namespace='reactive_planner',
		#	name='turtlebot3_map_publisher',
		#	output='screen',
		# ),
	])
