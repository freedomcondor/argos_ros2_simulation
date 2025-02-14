#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "drone/msg/waypoint.hpp"
#include "drone/msg/path.hpp"

class droneFlightSystem: public rclcpp::Node
{
public:
	droneFlightSystem() : Node("droneFlightSystemNode"),
		m_CurrentPosition(), m_CurrentOrientation()
	{
		auto PositionSensorSubscriberCallback =
			[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
				m_CurrentPosition = msg->position;
				m_CurrentOrientation = msg->orientation;
				m_CurrentYaw = atan2(2.0 * (m_CurrentOrientation.w * m_CurrentOrientation.z + m_CurrentOrientation.x * m_CurrentOrientation.y),
				                     1.0 - 2.0 * (m_CurrentOrientation.y * m_CurrentOrientation.y + m_CurrentOrientation.z * m_CurrentOrientation.z)
				                    );
			};
		m_PositionSensorSubscriber =
			this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10, PositionSensorSubscriberCallback);

		auto PathSubscriberCallback =
			[this](drone::msg::Path::UniquePtr msg) -> void {
				m_Path.clear();
				for(const auto& waypoint : msg->waypoints) {
					m_Path.push_back(waypoint);
				}
			};
		m_PathSubscriber =
			this->create_subscription<drone::msg::Path>("dronePath", 10, PathSubscriberCallback);

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

	void step()
	{
		//arrival check
		while (!m_Path.empty())
		{
			const auto& currentWaypoint = m_Path[0];
			double distance = sqrt(pow(m_CurrentPosition.x - currentWaypoint.point.x, 2) +
			                       pow(m_CurrentPosition.y - currentWaypoint.point.y, 2) +
			                       pow(m_CurrentPosition.z - currentWaypoint.point.z, 2)
			                      );
			double yaw_distance = 0;
			if (currentWaypoint.yaw_flag == true) yaw_distance = m_CurrentYaw - currentWaypoint.yaw;
			if ((distance < 0.1) && (yaw_distance < M_PI * 5 / 180)) // Assuming a threshold of 0.1 meters, 5 degree
				m_Path.erase(m_Path.begin()); // Remove the first waypoint from m_Path
			else
				break;
		}

		// go to the first point in path
		if (!m_Path.empty()) {
			const auto& currentWaypoint = m_Path[0];

			geometry_msgs::msg::Pose poseMessage;
			poseMessage.position = currentWaypoint.point;

			if (currentWaypoint.yaw_flag) {
				poseMessage.orientation.x = 0;
				poseMessage.orientation.y = 0;
				poseMessage.orientation.z = currentWaypoint.yaw;
				poseMessage.orientation.w = 0;
			} else {
				double yaw = atan2(currentWaypoint.point.y - m_CurrentPosition.y,
				                   currentWaypoint.point.x - m_CurrentPosition.x);
				poseMessage.orientation.x = 0;
				poseMessage.orientation.y = 0;
				poseMessage.orientation.z = yaw;
				poseMessage.orientation.w = 0;
			}

			m_PositionActuator->publish(poseMessage);
		}
	}

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_PositionSensorSubscriber;
	rclcpp::Subscription<drone::msg::Path>::SharedPtr m_PathSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_PositionActuator;
	geometry_msgs::msg::Point m_CurrentPosition;
	geometry_msgs::msg::Quaternion m_CurrentOrientation;
	double m_CurrentYaw;
	std::vector<drone::msg::Waypoint> m_Path;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto drone_node = std::make_shared<droneFlightSystem>();
	rclcpp::Rate loop_rate(10); // 10 Hz
	//rclcpp::spin(drone_node);
	while (rclcpp::ok())
	{
		rclcpp::spin_some(drone_node);
		drone_node->step();
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}