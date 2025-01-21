#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

class droneFlightSystem: public rclcpp::Node
{
public:
	droneFlightSystem() : Node("drone"),
		m_CurrentPosition(), m_CurrentOrientation()
	{
		auto PositionSensorSubscriberCallback =
			[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
				m_CurrentPosition = msg->position;
				m_CurrentOrientation = msg->orientation;
			};
		m_PositionSensorSubscriber =
			this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10, PositionSensorSubscriberCallback);

		m_PositionActuator = this->create_publisher<geometry_msgs::msg::Pose>("dronePoseActuator", 10);
	}

	void publishPosition(double x, double y, double z, double yaw)
	{
		geometry_msgs::msg::Pose poseMessage;
		poseMessage.position.x = x;
		poseMessage.position.y = y;
		poseMessage.position.z = z;
		poseMessage.orientation.x = 0;
		poseMessage.orientation.y = 0;
		poseMessage.orientation.z = yaw;  // abuse z as yaw
		poseMessage.orientation.w = 0;
		m_PositionActuator->publish(poseMessage);
	}

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_PositionSensorSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_PositionActuator;
	geometry_msgs::msg::Point m_CurrentPosition;
	geometry_msgs::msg::Quaternion m_CurrentOrientation;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto drone_node = std::make_shared<droneFlightSystem>();
	rclcpp::Rate loop_rate(10); // 10 Hz
	while (rclcpp::ok())
	{
		drone_node->publishPosition(
			drone_node->m_CurrentPosition.x + 1,
			drone_node->m_CurrentPosition.y,
			drone_node->m_CurrentPosition.z,
			0
		); // 发布位置
		rclcpp::spin_some(drone_node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}