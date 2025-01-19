#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

class droneFlightSystem: public rclcpp::Node
{
public:
  droneFlightSystem()
  : Node("drone"), position_(), orientation_()
  {
    auto topic_callback =
      [this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
        position_ = msg->position;
        orientation_ = msg->orientation;
        RCLCPP_INFO(this->get_logger(), "Received position: (%f, %f, %f)", 
                     position_.x, position_.y, position_.z);
        RCLCPP_INFO(this->get_logger(), "Received orientation: (%f, %f, %f, %f)", 
                     orientation_.x, orientation_.y, orientation_.z, orientation_.w);
      };
    subscription_ =
      this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10, topic_callback);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  geometry_msgs::msg::Point position_;
  geometry_msgs::msg::Quaternion orientation_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<droneFlightSystem>());
  rclcpp::shutdown();
  return 0;
}