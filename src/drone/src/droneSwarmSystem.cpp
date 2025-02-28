#include <vector>
using std::vector;
#include <string>
using std::string;

#include <memory>
#include <map>
using std::map;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "drone/msg/pose_sharing.hpp"
#include "drone/msg/waypoint.hpp"
#include "drone/msg/path.hpp"

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

class droneSwarmSystem: public rclcpp::Node
{
public:
	droneSwarmSystem() : Node("droneSwarmSystemNode")
	{
		// get my ID
		m_strMyID = this->get_namespace();
		if (m_strMyID[0] == '/') {
			m_strMyID = m_strMyID.substr(1); // Remove leading '/'
		}

		// declare position sharing publisher
		m_PoseSharingPublisher =
			this->create_publisher<drone::msg::PoseSharing>("/poseSharing", 10);

		// declare path publisher
		m_PathPublisher =
			this->create_publisher<drone::msg::Path>("dronePath", 10);

		// receive my pose, and publish it into position sharing with my ID
		m_PositionSensorSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10,
			[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
				m_CurrentTransform = CTransform(*msg);

				drone::msg::PoseSharing poseSharingMsg;
				poseSharingMsg.id = m_strMyID;
				poseSharingMsg.pose = *msg;
				m_PoseSharingPublisher->publish(poseSharingMsg);
			}
		);

		// 订阅PoseSharing消息
		m_PoseSharingSubscriber = this->create_subscription<drone::msg::PoseSharing>("/poseSharing", 10,
			[this](const drone::msg::PoseSharing::SharedPtr msg) -> void{
				m_SwarmPoses[msg->id] = CTransform(msg->pose);
			}
		);
	}

	void step() {
		for (const auto& swarmPose : m_SwarmPoses) {
			const string& id = swarmPose.first;
			const CTransform& pose = swarmPose.second;

			// Check if the current drone is drone1 (assuming drone1 has a specific ID)
			if ((m_strMyID == "drone2") && (id == "drone1")) {
				CTransform offset = CTransform(
					CVector3(3, 0, 0),
					CQuaternion()
				);

				CTransform newPosition = pose * offset;

				double Yaw, Roll, Pitch;
				newPosition.m_Orientation.ToEulerAngles(Yaw, Roll, Pitch);

				// Publish the new position for drone2
				drone::msg::Waypoint waypoint;
				waypoint.point.x = newPosition.m_Position.GetX();
				waypoint.point.y = newPosition.m_Position.GetY();
				waypoint.point.z = newPosition.m_Position.GetZ();
				waypoint.yaw_flag = true;
				waypoint.yaw = Yaw;

				vector<drone::msg::Waypoint> waypoints = {waypoint};
				drone::msg::Path path;
				path.waypoints = waypoints;
				m_PathPublisher->publish(path);
			}
		}
	}

private:
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_PositionSensorSubscriber;
	rclcpp::Publisher<drone::msg::PoseSharing>::SharedPtr m_PoseSharingPublisher;
	rclcpp::Publisher<drone::msg::Path>::SharedPtr m_PathPublisher;

	CTransform m_CurrentTransform;
	string m_strMyID;
	rclcpp::Subscription<drone::msg::PoseSharing>::SharedPtr m_PoseSharingSubscriber;
	std::map<string, CTransform> m_SwarmPoses;  // 存储所有无人机的位置和姿态
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto swarm_node = std::make_shared<droneSwarmSystem>();
	rclcpp::Rate loop_rate(10); // 10 Hz
	//rclcpp::spin(drone_node);
	while (rclcpp::ok())
	{
		rclcpp::spin_some(swarm_node);
		swarm_node->step();
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}