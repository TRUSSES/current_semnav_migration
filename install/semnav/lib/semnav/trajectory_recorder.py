#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import csv

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('output_file', 'trajectory0.csv')

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.sub_twist = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        self.sub_odom  # Prevent unused variable warning
        self.sub_twist

        # Store trajectory in "/data/trajectory*.csv", twist cmds from planner in '/data/twist*.csv'
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        filename = get_package_share_directory("semnav") + '/data/' + output_file
        print('recording in ' + filename)

        self.trajectory_csv = open(filename, 'w')
        self.writer = csv.writer(self.trajectory_csv)
        self.writer.writerow(['timestamp (sec)', 'x', 'y', 'linear_x', 'angular_z'])
        print('lin_x')

        self.latest_cmd_vel = Twist()  # store latest twist msg

    def odom_callback(self, msg):
        timestamp = msg.header.stamp.sec
        position = msg.pose.pose.position
        linear_x = self.latest_cmd_vel.linear.x
        angular_z = self.latest_cmd_vel.angular.z

        self.writer.writerow([timestamp, position.x, position.y, linear_x, angular_z])
    
    def twist_callback(self, msg):
        self.latest_cmd_vel = msg

    def __del__(self):
        # Close csv files when node is destroyed
        self.trajectory_csv.close()

def main(args=None):
    rclpy.init(args=args)
    trajectory_recorder = TrajectoryRecorder()
    rclpy.spin(trajectory_recorder)
    trajectory_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()