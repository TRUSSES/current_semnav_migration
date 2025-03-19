#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import csv

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('output_file', 'trajectory0.csv')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Store trajectory in "/data/trajectory.csv"
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        file_name = get_package_share_directory("semnav") + '/data/' + output_file
        print('recording in ' + file_name)
        self.trajectory_csv = open(file_name, 'w')
        self.writer = csv.writer(self.trajectory_csv)
        self.writer.writerow(['timestamp (sec)', 'x', 'y'])

    def odom_callback(self, msg):
        timestamp = msg.header.stamp.sec
        position = msg.pose.pose.position

        self.writer.writerow([timestamp, position.x, position.y])

    def __del__(self):
        # Close csv file when node is destroyed
        self.trajectory_csv.close()

def main(args=None):
    rclpy.init(args=args)
    trajectory_recorder = TrajectoryRecorder()
    rclpy.spin(trajectory_recorder)
    trajectory_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()