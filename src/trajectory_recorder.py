#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import csv

import numpy as np
if not hasattr(np, 'float'):
    np.float = float # resolve tf_transformations np.float issue
from tf_transformations import euler_from_quaternion

class TrajectoryRecorder(Node):
    def __init__(self):
        print('initializing trajectory recorder node')

        super().__init__('trajectory_recorder')

        self.declare_parameter('output_file', 'trajectory.csv')

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Store trajectory in "/data/trajectory*.csv"
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        filename = get_package_share_directory("semnav") + '/data/' + output_file
        print('recording in ' + filename)

        self.trajectory_csv = open(filename, 'w')
        self.writer = csv.writer(self.trajectory_csv)
        self.writer.writerow(['index', 'x', 'y', 'orientation'])

        self.row = 0 # row index in output file
        self.record_rate = 10 # Hz
        self.record_interval = Duration(seconds=1.0 / self.record_rate)
        self.last_update_time = self.get_clock().now()

    def odom_callback(self, msg):
        if (self.get_clock().now() - self.last_update_time < self.record_interval) :
            return
        
        timestamp = msg.header.stamp.sec
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.writer.writerow([self.row, position.x, position.y, yaw])
        self.row += 1
        self.last_update_time = self.get_clock().now()
    

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