#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "drone/msg/waypoint.hpp"
#include "drone/msg/path.hpp"

#include "mathlib/math/vector3.h"
#include "mathlib/math/quaternion.h"
using namespace swarmMathLib;

class droneFlightSystem: public rclcpp::Node
{
public:
	droneFlightSystem() : Node("droneFlightSystemNode"),
		m_CurrentPosition(), m_CurrentOrientation()
	{
		m_PositionSensorSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10,
			[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
				m_CurrentPosition = CVector3(
					msg->position.x,
					msg->position.y,
					msg->position.z
				);
				m_CurrentOrientation = CQuaternion(
					msg->orientation.w,
					msg->orientation.x,
					msg->orientation.y,
					msg->orientation.z
				);
				m_CurrentYaw = atan2(2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y),
				                     1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z)
				                    );
			}
		);

		m_PathSubscriber = this->create_subscription<drone::msg::Path>("dronePath", 10,
			[this](drone::msg::Path::UniquePtr msg) -> void {
				m_Path.clear();
				for(const auto& waypoint : msg->waypoints) {
					m_Path.push_back(waypoint);
				}
			}
		);

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
			CVector3 currentWaypointPosition = CVector3(
				m_Path[0].point.x,
				m_Path[0].point.y,
				m_Path[0].point.z
			);
			double distance = (m_CurrentPosition - currentWaypointPosition).Length();
			double yawDistance = 0;
			if (currentWaypoint.yaw_flag == true) yawDistance = m_CurrentYaw - currentWaypoint.yaw;
			if ((distance < m_PositionArrivalCheck) && (yawDistance < m_YawArrivalCheck)) // Assuming a threshold of 0.1 meters, 5 degree
				m_Path.erase(m_Path.begin()); // Remove the first waypoint from m_Path
			else
				break;
		}

		// go to the first point in path
		if (!m_Path.empty()) {
			const auto& currentWaypoint = m_Path[0];
			CVector3 currentWaypointPosition = CVector3(
				m_Path[0].point.x,
				m_Path[0].point.y,
				m_Path[0].point.z
			);
			/*
			CVector3 offset = (currentWaypointPosition - m_CurrentPosition).Normalize() * 15;
			CVector3 flyTo = m_CurrentPosition + offset;

			geometry_msgs::msg::Pose poseMessage;
			poseMessage.position.x = flyTo.GetX();
			poseMessage.position.y = flyTo.GetY();
			poseMessage.position.z = flyTo.GetZ();
			*/
			geometry_msgs::msg::Pose poseMessage;
			poseMessage.position.x = currentWaypointPosition.GetX();
			poseMessage.position.y = currentWaypointPosition.GetY();
			poseMessage.position.z = currentWaypointPosition.GetZ();

			if (currentWaypoint.yaw_flag) {
				poseMessage.orientation.x = 0;
				poseMessage.orientation.y = 0;
				poseMessage.orientation.z = currentWaypoint.yaw;
				poseMessage.orientation.w = 0;
			} else {
				double yaw = atan2(currentWaypoint.point.y - m_CurrentPosition.GetY(),
								currentWaypoint.point.x - m_CurrentPosition.GetX());
				poseMessage.orientation.x = 0;
				poseMessage.orientation.y = 0;
				poseMessage.orientation.z = yaw;
				poseMessage.orientation.w = 0;
			}

			m_PositionActuator->publish(poseMessage);
		}
		else {
			geometry_msgs::msg::Pose poseMessage;
			poseMessage.position.x = m_CurrentPosition.GetX();
			poseMessage.position.y = m_CurrentPosition.GetY();
			poseMessage.position.z = m_CurrentPosition.GetZ();

			double Yaw, Roll, Pitch;
			m_CurrentOrientation.ToEulerAngles(Yaw, Roll, Pitch);
			poseMessage.orientation.x = 0;
			poseMessage.orientation.y = 0;
			poseMessage.orientation.z = Yaw;
			poseMessage.orientation.w = 0;

			m_PositionActuator->publish(poseMessage);
		}
	}

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_PositionSensorSubscriber;
	rclcpp::Subscription<drone::msg::Path>::SharedPtr m_PathSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_PositionActuator;

	CVector3 m_CurrentPosition;
	CQuaternion m_CurrentOrientation;
	double m_CurrentYaw;

	double m_PositionArrivalCheck = 0.1;
	double m_YawArrivalCheck = M_PI * 5 / 180;

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