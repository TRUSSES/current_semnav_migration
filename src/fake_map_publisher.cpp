#include <rclcpp/rclcpp.hpp>
#include <object_pose_interface_msgs/msg/semantic_map_object_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "../data/scripts/generate_map.h" // for "get_polygons(file)"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <vector>
#include <cmath>

class FakeMapPublisherNode : public rclcpp::Node {
public:
  FakeMapPublisherNode() : rclcpp::Node("fake_map_publisher") {
    this->declare_parameter("pub_semantic_topic", "semantic_map");
    this->declare_parameter("pub_transform_topic", "world_transform");
    this->declare_parameter("obstacle_file", "4x4rect.csv");
    this->declare_parameter("spawn_x", 0.0);
    this->declare_parameter("spawn_y", 0.0);

    pub_semantic_topic_ = this->get_parameter("pub_semantic_topic").as_string();
    pub_transform_topic_ = this->get_parameter("pub_transform_topic").as_string();
    obstacle_file_ = this->get_parameter("obstacle_file").as_string();
    spawn_x = this->get_parameter("spawn_x").as_double();
    spawn_y = this->get_parameter("spawn_y").as_double();

    std::cout << "obstacle_file: " << obstacle_file_ << std::endl;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.depth = 1; 
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
    
    pub_semantic_map_ = this->create_publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>(
        pub_semantic_topic_, qos);
    pub_world_transform_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        pub_transform_topic_, qos);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&FakeMapPublisherNode::odom_callback, this, std::placeholders::_1));

    // Get polygon coordinates from CSV data and create "model.sdf" to visualize in Gazebo.
    std::string share_directory = ament_index_cpp::get_package_share_directory("semnav");
    std::string filename = share_directory + "/data/obstacle_maps/" + obstacle_file_;

    test_polygons = get_polygons(filename);
    generate_sdf(test_polygons);

    timer_ = this->create_wall_timer(std::chrono::nanoseconds(100000), std::bind(&FakeMapPublisherNode::publish_map, this));
    transform_timer_ = this->create_wall_timer(std::chrono::nanoseconds(100000), std::bind(&FakeMapPublisherNode::publish_world_frame, this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_z_ = msg->pose.pose.position.z;
  }

  void publish_world_frame() {
    // world frame must be translated by spawn point to align with odom frame
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    //transformStamped.transform.translation.x = 0.0;
    //transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.x = spawn_x;
    transformStamped.transform.translation.y = spawn_y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transformStamped);
    pub_world_transform_->publish(transformStamped);
  }

  void publish_map() {
    /* Limit map publish count if it is static
    if (msg_count >= msg_limit) {
      return;
    } else {
      msg_count++;
    }
    */

    object_pose_interface_msgs::msg::SemanticMapObjectArray polygon_list_msg;

    double frame_center_x = 0.0;
    double frame_center_y = 0.0;

    for (int i = 0; i < test_polygons.size(); i++) {
      polygon_list_msg.objects.push_back(
        populate_polygon_msg(test_polygons[i], frame_center_x, frame_center_y, robot_z_));
    }

    pub_semantic_map_->publish(polygon_list_msg);
  }

  object_pose_interface_msgs::msg::SemanticMapObject populate_polygon_msg(
    const std::vector<std::vector<double>>& polygon_in,
    double x, double y, double z) {

    object_pose_interface_msgs::msg::SemanticMapObject polygon_out;

    for (const auto& point : polygon_in) {
      geometry_msgs::msg::Point32 point_new;
      point_new.x = point[0];
      point_new.y = point[1];
      polygon_out.polygon2d.polygon.points.push_back(point_new);
    }

    polygon_out.pose.header.frame_id = "map";
    polygon_out.pose.pose.position.x = x;
    polygon_out.pose.pose.position.y = y;
    polygon_out.pose.pose.position.z = z;
    polygon_out.pose.pose.orientation.x = 2.0;
    polygon_out.pose.pose.orientation.y = -2.0;
    polygon_out.pose.pose.orientation.z = 0.0;
    polygon_out.pose.pose.orientation.w = 1.0;

    return polygon_out;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::Publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>::SharedPtr pub_semantic_map_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_world_transform_;  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  std::string pub_semantic_topic_;
  std::string pub_transform_topic_;  
  std::string obstacle_file_;
  double spawn_x;
  double spawn_y;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<std::vector<std::vector<double>>> test_polygons;

  double robot_x_ = 0.0, robot_y_ = 0.0, robot_z_ = 0.0;
  int msg_limit = 10, msg_count = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeMapPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
