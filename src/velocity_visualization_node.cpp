#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <object_pose_interface_msgs/msg/semantic_map_object_array.hpp>
#include <vector>
#include <chrono> // ms literals for Duration
#include <memory>

class VectorVisualizationNode : public rclcpp::Node {
    public:
        VectorVisualizationNode() : rclcpp::Node(std::string("visualization_node")) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Visualizer] Visualization Node Initialized");
            
            this->declare_parameter("pub_pose_topic", "/odom");
            this->declare_parameter("pub_marker_topic", "/marker_array"); // For vector field visualization
            this->declare_parameter("sub_twist_topic", "/cmd_vel");
            //this->declare_parameter("sub_semantic_topic", "/semantic_map"); // For obstacle polygons

            this->declare_parameter("x_min", -10.0);
            this->declare_parameter("x_max", 10.0);
            this->declare_parameter("y_min", -10.0);
            this->declare_parameter("y_max", 10.0);
            this->declare_parameter("n_points", 20); // Number of grid points graphed between bounds
            this->declare_parameter("wait_time", 0.1);

            // Topic names
            pub_pose_topic_ = this->get_parameter("pub_pose_topic").as_string();
            pub_marker_topic_ = this->get_parameter("pub_marker_topic").as_string();
            sub_twist_topic_ = this->get_parameter("sub_twist_topic").as_string();
            //sub_semantic_topic_ = this->get_parameter("sub_semantic_topic").as_string();

            pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_marker_topic_, 10);
            
            // Custom QoS for compatibility with navigation node message_filters::Subscriber
            rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

            pub_pose_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_pose_topic_, qos_profile);
            
            sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
                sub_twist_topic_,
                10,
                std::bind(&VectorVisualizationNode::twist_callback, this, std::placeholders::_1)
            );
            //sub_semantic_ = this->create_subscription<object_pose_interface_msgs::msg::SemanticMapObjectArray>(sub_semantic_topic_, 1);

            // Grid parameters
            x_min_ = this->get_parameter("x_min").as_double();
            x_max_ = this->get_parameter("x_max").as_double();
            y_min_ = this->get_parameter("y_min").as_double();
            y_max_ = this->get_parameter("y_max").as_double();
            n_points_ = this->get_parameter("n_points").as_int();
            wait_time_ = this->get_parameter("wait_time").as_double();
        
            current_twist_ = nullptr;
            new_twist_received_ = true; // Allows timer callback to publish first grid point

            // Publish one grid pose at a time
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&VectorVisualizationNode::publish_grid_pose, this)
            );
        }
    
    private:
        // Parameters
        std::string pub_pose_topic_;
        std::string pub_marker_topic_;
        std::string sub_twist_topic_;
        std::string sub_semantic_topic_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pose_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
        //rclcpp::Subscription<object_pose_interface_msgs::msg::SemanticMapObjectArray>::SharedPtr sub_semantic_;

        double x_min_, x_max_, y_min_, y_max_, wait_time_;
        int n_points_;

        std::vector<std::pair<double, double>> grid_points_;
        int current_point_index_;
        std::shared_ptr<geometry_msgs::msg::Twist> current_twist_;

        visualization_msgs::msg::MarkerArray marker_array_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool new_twist_received_;

        void check_for_twist() {
            // timer callback, not used
            if (current_twist_) {
                auto current_point = grid_points_[current_point_index_];
                add_marker(current_point.first, current_point.second, *current_twist_);
                current_twist_.reset();
            }
        }

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "[Visualizer] In twist callback.");
            current_twist_ = msg;
            //new_twist_received_ = true;
        }
        
        void publish_grid_pose() {
            if (!new_twist_received_) {
                return;
            }

            if (grid_points_.empty()) {
                initialize_grid_points();
                current_point_index_ = 0;
            }

            if (current_point_index_ >= grid_points_.size()) {
                RCLCPP_INFO(this->get_logger(), "[Visualizer] Added all grid points.");
                pub_marker_->publish(marker_array_);
                RCLCPP_INFO(this->get_logger(), "[Visualizer] Published marker array.");
                //timer_->cancel();
                return;
            }

            // Publish next pose.
            auto current_point = grid_points_[current_point_index_];
            publish_pose(current_point.first, current_point.second);

            // Wait for twist command.
            auto start_time = this->now();
            auto wait_timeout = rclcpp::Duration::from_seconds(wait_time_);

            while (rclcpp::ok() && (this->now() - start_time) < wait_timeout) {
                if (current_twist_) {
                    RCLCPP_INFO(this->get_logger(), "has current twist");
                    add_marker(current_point.first, current_point.second, *current_twist_);
                    break;
                }

            }

            // Wait timed out
            if (!current_twist_) {
                RCLCPP_WARN(this->get_logger(), "Wait timed out for (%f, %f)",
                    current_point.first, current_point.second);
            } else {
                current_twist_.reset();
            }

            current_point_index_++;
        }

        void initialize_grid_points() {
            double x_spacing = (x_max_ - x_min_) / (n_points_ - 1);
            double y_spacing = (y_max_ - y_min_) / (n_points_ - 1);
            for (double x = x_min_; x < x_max_; x += x_spacing) {
                for (double y = y_min_; y < y_max_; y += y_spacing) {
                    grid_points_.emplace_back(x, y);
                }
            }
        }

        void publish_pose(double x, double y) {
            auto pose = geometry_msgs::msg::Pose();
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;

            // Need an Odometry msg
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = this->now();
            odom.header.frame_id = "map";
            odom.pose.pose = pose;

            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.linear.z = 0.0;
            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = 0.0;

            pub_pose_->publish(odom);
            RCLCPP_INFO(this->get_logger(), "[Visualizer] Published odom (%f, %f)", x, y);
        }

        void add_marker(double x, double y, const geometry_msgs::msg::Twist &twist) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = "map";
            marker.ns = "vector_field";
            marker.id = current_point_index_;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;

            double robot_radius = 0.3;
            double vx = twist.linear.x; // robot frame should match world frame

            // Manually set orientation components
            tf2::Quaternion q;
            
            // To plot angular velocity component only
            q.setRPY(0, 0, twist.angular.z);

            // To plot linear velocity component only
            /*
            if (vx >= 0) {
                q.setRPY(0, 0, 0); // along +x axis
            } else {
                q.setRPY(0, 0, 3.14); // along -x axis
            }*/
            
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            // For plot of angular velocity direction only
            // marker.scale.x = (x_max_ - x_min_) / n_points_; // scale shaft length by workspace

            // For plot of linear velocity magnitude only
            marker.scale.x = vx * 2 * (x_max_ - x_min_) / n_points_;
            marker.scale.y = marker.scale.x / 10.0; // arrow width
            marker.scale.z = marker.scale.x / 10.0; // arrow height

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            
            marker_array_.markers.push_back(marker);
            RCLCPP_INFO(this->get_logger(), "[Visualizer] Added marker.");
        }
        
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Add node instead of spin.
    auto node = std::make_shared<VectorVisualizationNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}