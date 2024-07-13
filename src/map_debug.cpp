// License: MIT (modified) - See original file for details

#include <rclcpp/rclcpp.hpp>
#include <object_pose_interface_msgs/msg/semantic_map_object_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>

using namespace std::chrono::milliseconds;

class MapDebugNode : public rclcpp::Node {
public:
  MapDebugNode() : Node("map_debug") {
    this->declare_parameter("pub_semantic_topic", "semantic_map");

    pub_semantic_topic_ = this->get_parameter("pub_semantic_topic").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.depth = 1; 
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    pub_semantic_map_ = this->create_publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>(
        pub_semantic_topic_, qos_profile);

    timer_ = this->create_wall_timer(100ms, std::bind(&MapDebugNode::publish_map, this));
  }

private:
  void publish_map() {
    std::vector<std::vector<double>> polygon1 = {{0.0, 0.0}, {0.5, 0.0}, {0.5, -1.0}, {1.5, -1.0}, {1.5, 1.0}, 
                                                {-1.0, 1.0}, {-1.0, -3.0}, {3.0, -3.0}, {3.0, 1.0}, {2.0, 1.0}, 
                                                {2.0, -2.0}, {0.0, -2.0}, {0.0, 0.0}};

    object_pose_interface_msgs::msg::SemanticMapObjectArray polygon_list_msg;
    polygon_list_msg.objects.push_back(populate_polygon_msg(polygon1));
    pub_semantic_map_->publish(polygon_list_msg);
  }

  object_pose_interface_msgs::msg::SemanticMapObject populate_polygon_msg(const std::vector<std::vector<double>>& polygon_in) {
    object_pose_interface_msgs::msg::SemanticMapObject polygon_out;
    for (const auto& point : polygon_in) {
      geometry_msgs::msg::Point32 point_new;
      point_new.x = point[0];
      point_new.y = point[1];
      polygon_out.polygon2d.polygon.points.push_back(point_new);
    }
    return polygon_out;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<object_pose_interface_msgs::msg::SemanticMapObjectArray>::SharedPtr pub_semantic_map_;

  // Parameters
  std::string pub_semantic_topic_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapDebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}