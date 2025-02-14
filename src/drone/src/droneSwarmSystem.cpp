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
		m_PositionSensorSubscriber =
			this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10,
				[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
					m_CurrentPosition = CVector3(
						msg->position.x,
						msg->position.y,
						msg->position.z
					);

					drone::msg::PoseSharing poseSharingMsg;
					poseSharingMsg.id = m_strMyID;
					poseSharingMsg.pose = *msg; // Assuming msg contains the pose
					m_PoseSharingPublisher->publish(poseSharingMsg);
				}
			);

		// 订阅PoseSharing消息
		m_PoseSharingSubscriber = 
			this->create_subscription<drone::msg::PoseSharing>("/poseSharing", 10,
				[this](const drone::msg::PoseSharing::SharedPtr msg) -> void{
					m_SwarmPoses[msg->id] = msg->pose;
				}
			);
	}

	void step() {
		for (const auto& swarmPose : m_SwarmPoses) {
			const auto& id = swarmPose.first;
			const auto& pose = swarmPose.second;

			// Check if the current drone is drone1 (assuming drone1 has a specific ID)
			if ((m_strMyID == "drone2") && (id == "drone1")) {
				// Generate a circular path around drone1 for drone2
				double radius = 5.0; // radius of the circle
				double angularSpeed = 0.1; // speed of rotation
				static double angle = 0.0; // static to retain value between calls

				// Calculate new position for drone2
				double newX = pose.position.x + radius * cos(angle);
				double newY = pose.position.y + radius * sin(angle);
				double newZ = pose.position.z; // maintain the same height

				// Update angle for the next position
				angle += angularSpeed;

				// Publish the new position for drone2
				drone::msg::Waypoint waypoint;
				waypoint.point.x = newX;
				waypoint.point.y = newY;
				waypoint.point.z = newZ;
				waypoint.yaw_flag = true;
				waypoint.yaw = 0;

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

	CVector3 m_CurrentPosition;
	string m_strMyID;
	rclcpp::Subscription<drone::msg::PoseSharing>::SharedPtr m_PoseSharingSubscriber;
	std::map<string, geometry_msgs::msg::Pose> m_SwarmPoses;  // 存储所有无人机的位置和姿态
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