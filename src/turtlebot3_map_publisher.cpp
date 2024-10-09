#include <reactive_planner_lib.h>
#include <object_pose_interface_msgs/msg/semantic_map_object_array.hpp>
#include <example_interfaces/msg/u_int32.hpp>

class SemanticMapNode : public rclcpp::Node {
public:
  SemanticMapNode() : Node("semantic_map_node") {
    sub_laser.subscribe(this, "/scan");
    sub_robot.subscribe(this, "/odom");
    sync_ = std::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>>>(
                            message_filters::sync_policies::ApproximateTime<
                            sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>(10),
                            sub_laser, sub_robot);
    
    sync_->registerCallback(std::bind(&SemanticMapNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    semantic_map_pub_ = this->create_publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>(
        "/semantic_map", 10);
  }

private:
  void callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {

    object_pose_interface_msgs::msg::SemanticMapObjectArray semantic_map;
    semantic_map.header.stamp = this->now();
    semantic_map.header.frame_id = scan_msg->header.frame_id;  // Use LIDAR frame

    // 1. Extract keypoints from LaserScan (LIDAR) message
    object_pose_interface_msgs::msg::SemanticMapObject map_object;
    map_object.keypoints = extract_keypoints(scan_msg);
    
    // 2. Use the pose information from the odometry message
    map_object.pose = extract_pose(odom_msg);

    semantic_map.objects.push_back(map_object);
    semantic_map_pub_->publish(semantic_map);
  }

  geometry_msgs::msg::PolygonStamped extract_keypoints(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg) {
    geometry_msgs::msg::PolygonStamped keypoints_msg;
    keypoints_msg.header.stamp = this->now();
    keypoints_msg.header.frame_id = scan_msg->header.frame_id;  // Same frame as LIDAR
    
    // Convert the scan data to 2D points (keypoints)
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      float range = scan_msg->ranges[i];
      
      // Ignore invalid ranges (outside of min/max range)
      if (range < scan_msg->range_min || range > scan_msg->range_max) {
        continue;
      }

      float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

      float x = range * cos(angle);
      float y = range * sin(angle);

      geometry_msgs::msg::Point32 point;
      point.x = x;
      point.y = y;
      point.z = 0.0;
      keypoints_msg.polygon.points.push_back(point);
    }

    return keypoints_msg;
  }

  geometry_msgs::msg::PoseStamped extract_pose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = odom_msg->header.stamp;
    pose_msg.header.frame_id = odom_msg->header.frame_id;  

    pose_msg.pose = odom_msg->pose.pose;

    return pose_msg;
  }

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_laser;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_robot;
  std::shared_ptr<message_filters::Synchronizer<
                    message_filters::sync_policies::ApproximateTime<
                    sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>>> sync_;
  rclcpp::Publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>::SharedPtr semantic_map_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SemanticMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
