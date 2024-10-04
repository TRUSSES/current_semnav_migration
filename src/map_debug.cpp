// License: MIT (modified) - See original file for details

#include <rclcpp/rclcpp.hpp>
#include <object_pose_interface_msgs/msg/semantic_map_object_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"

#include <vector>

class MapDebugNode : public rclcpp::Node {
public:
  MapDebugNode() : rclcpp::Node(std::string("map_debug")) {
    this->declare_parameter("pub_semantic_topic", "semantic_map");
    this->declare_parameter("pub_transform_topic", "world_transform");  // New parameter for transform topic

    pub_semantic_topic_ = this->get_parameter("pub_semantic_topic").as_string();
    pub_transform_topic_ = this->get_parameter("pub_transform_topic").as_string();  // Get the transform topic parameter

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.depth = 1; 
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
    
    pub_semantic_map_ = this->create_publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>(
        pub_semantic_topic_, qos);
    
    pub_world_transform_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        pub_transform_topic_, qos);  // Publisher for the world transform
    
    timer_ = this->create_wall_timer(std::chrono::nanoseconds(100000), std::bind(&MapDebugNode::publish_map, this));
    
    // Publish the world frame on a timer
    transform_timer_ = this->create_wall_timer(std::chrono::nanoseconds(100000), std::bind(&MapDebugNode::publish_world_frame, this));
  }

private:
  void publish_world_frame() {
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transformStamped;

    // Set the translation (position) and rotation (orientation) for the world frame
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";        // World frame
    transformStamped.child_frame_id = "odom";        // Local odometry frame

    // Set the translation values (0,0,0) for the map frame in this example
    transformStamped.transform.translation.x = 0.0;  
    transformStamped.transform.translation.y = 0.0;  
    transformStamped.transform.translation.z = 0.0;  

    // Set the rotation to identity (no rotation)
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    // Publish the transform to the new topic
    tf_broadcaster_->sendTransform(transformStamped);
    pub_world_transform_->publish(transformStamped);
  }

  void publish_map() {
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Navigation] Semantic Map Published");
    std::vector<std::vector<double>> polygon1 = {
      {0.0, 0.0},   
      {0.5, 0.0},  
      {0.5, 0.5},   
      {0.0, 0.5},  
      {0.0, 0.0}    
  };

    std::vector<std::vector<double>> polygon2 = {
      {0.0, 0.0},   
      {2, 0.0},  
      {2, 2},   
      {0.0, 2},  
      {0.0, 0.0}    
  };

    object_pose_interface_msgs::msg::SemanticMapObjectArray polygon_list_msg;
    // polygon_list_msg.objects.push_back(populate_polygon_msg(polygon1, -0.5, -0.5, 0.0));
    // polygon_list_msg.objects.push_back(populate_polygon_msg(polygon2, -2, -2, 0.0));
    pub_semantic_map_->publish(polygon_list_msg);
  }

  object_pose_interface_msgs::msg::SemanticMapObject populate_polygon_msg(
    const std::vector<std::vector<double>>& polygon_in,
    double x, double y, double z) {
  
    object_pose_interface_msgs::msg::SemanticMapObject polygon_out;

    // Populate the polygon points
    for (const auto& point : polygon_in) {
      geometry_msgs::msg::Point32 point_new;
      point_new.x = point[0];
      point_new.y = point[1];
      polygon_out.polygon2d.polygon.points.push_back(point_new);
    }
    
    // Set the frame_id of the polygon object to "map"
    polygon_out.pose.header.frame_id = "map";
    
    // Set the pose using the provided x, y, z coordinates
    polygon_out.pose.pose.position.x = x;
    polygon_out.pose.pose.position.y = y;
    polygon_out.pose.pose.position.z = z;
    
    // Optionally set the orientation to default (no rotation)
    polygon_out.pose.pose.orientation.x = 0.0;
    polygon_out.pose.pose.orientation.y = 0.0;
    polygon_out.pose.pose.orientation.z = 0.0;
    polygon_out.pose.pose.orientation.w = 1.0; // Identity quaternion

    return polygon_out;
  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;  // Timer for publishing the world frame transform
  rclcpp::Publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>::SharedPtr pub_semantic_map_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_world_transform_;  // Publisher for the world transform

  // Parameters
  std::string pub_semantic_topic_;
  std::string pub_transform_topic_;  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapDebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
