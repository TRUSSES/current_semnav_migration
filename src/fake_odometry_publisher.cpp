// MIT License (modified)
// Copyright (c) 2020 The Trustees of the University of Pennsylvania
// Authors:
// Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this **file** (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class FakeOdometryPublisherNode : public rclcpp::Node
{
public:
    // Constructor
    FakeOdometryPublisherNode() : Node("fake_odometry_publisher_node")
    {
        // Declare parameters
        this->declare_parameter("pub_odom_topic", "fake_odom");

        // Get parameter values
        pub_odom_topic_ = this->get_parameter("pub_odom_topic").as_string();

        // Initialize publishers
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(50)));

        // Odometry data
        double odom_frequency = 30;

        // Odometry publishing loop
        rclcpp::WallRate rate(odom_frequency);
        while (rclcpp::ok())
        {
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->now();
            odom_msg.child_frame_id = "robot";
            odom_msg.pose.pose.position.x = -1.5;
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.orientation.w = 1.0;

            publish_odom(odom_msg);
            rate.sleep();
        }
    }

    void publish_odom(nav_msgs::msg::Odometry odom_data)
    {
        pub_odom_->publish(odom_data);
        return;
    }

private:
    // Parameters
    std::string pub_odom_topic_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
};

int main(int argc, char **argv)
{
    // ROS 2 initialization
    rclcpp::init(argc, argv);

    // Create the node instance
    auto node = std::make_shared<FakeOdometryPublisherNode>();

    // Spin the node
    rclcpp::spin(node);

    // ROS 2 cleanup
    rclcpp::shutdown();

    return 0;
}