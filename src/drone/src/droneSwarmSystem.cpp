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

		m_VelocityActuatorPublisher =
			this->create_publisher<geometry_msgs::msg::Pose>("droneVelocityActuator", 10);

		// receive my pose, and publish it into position sharing with my ID
		m_PositionSensorSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10,
			[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
				m_CurrentTransform = CTransform(*msg);
			}
		);

		// declare position sharing publisher and subscriber
		m_PoseSharingPublisher =
			this->create_publisher<drone::msg::PoseSharing>("/poseSharing", 10);

		this->declare_parameter("distance_threshold", 5.0);
		this->declare_parameter("target_distance", 3.0);

		m_PoseSharingSubscriber = this->create_subscription<drone::msg::PoseSharing>("/poseSharing", 10,
			[this](const drone::msg::PoseSharing::SharedPtr msg) -> void{
				if (msg->id == m_strMyID) return;

				double distanceThreshold;
				this->get_parameter("distance_threshold", distanceThreshold);

				CTransform neighbour = CTransform(msg->pose);
				if ((neighbour.m_Position - m_CurrentTransform.m_Position).Length() < distanceThreshold)
					m_SwarmPoses[msg->id] = CTransform(msg->pose) - m_CurrentTransform;
				else
					m_SwarmPoses.erase(msg->id); // Remove the drone's pose if it's too far
			}
		);

		// step timer, call step() in a frequency
		m_Timer = this->create_wall_timer(
			std::chrono::milliseconds(200), // ms
			[this]() { this->step(); } // 定时器回调
		);
	}

	void step() {
		// share myself
		drone::msg::PoseSharing poseSharingMsg;
		poseSharingMsg.id = m_strMyID;
		poseSharingMsg.pose = m_CurrentTransform.ToGeometryMsgPose();
		m_PoseSharingPublisher->publish(poseSharingMsg);

		// -------------------------------------------------------------
		CVector3 vTotal = CVector3();
		double targetDistance;
		this->get_parameter("target_distance", targetDistance);

		for (const auto& swarmPose : m_SwarmPoses) {
			const string& id = swarmPose.first;
			const CTransform& pose = swarmPose.second;
			if ((m_strMyID != id) && (pose.m_Position.Length() != 0)) {
				CVector3 v = CVector3(pose.m_Position).Normalize() * (pose.m_Position.Length() - targetDistance) * 0.06;
				if (v.Length() > 0.5) v = v.Normalize() * 0.5;
				vTotal += v;
			}
		}

		setVelocity(vTotal);
	}

	void setVelocity(CVector3 v) {
		setVelocity(v.GetX(), v.GetY(), v.GetZ(), 0);
	}

	void setVelocity(double x, double y, double z, double th) {
		CVector3 globalVelocity = CVector3(x, y, z).Rotate(m_CurrentTransform.m_Orientation);
		geometry_msgs::msg::Pose velocityMessage;
		velocityMessage.position.x = globalVelocity.GetX();
		velocityMessage.position.y = globalVelocity.GetY();
		velocityMessage.position.z = globalVelocity.GetZ();
		velocityMessage.orientation.x = 0;
		velocityMessage.orientation.y = 0;
		velocityMessage.orientation.z = th;
		velocityMessage.orientation.w = 0;

		m_VelocityActuatorPublisher->publish(velocityMessage);
	}

private:
	string m_strMyID;
	CTransform m_CurrentTransform;
	std::map<string, CTransform> m_SwarmPoses;

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_PositionSensorSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_VelocityActuatorPublisher;

	rclcpp::Subscription<drone::msg::PoseSharing>::SharedPtr m_PoseSharingSubscriber;
	rclcpp::Publisher<drone::msg::PoseSharing>::SharedPtr m_PoseSharingPublisher;

	rclcpp::TimerBase::SharedPtr m_Timer; // 定时器
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto swarm_node = std::make_shared<droneSwarmSystem>();
	rclcpp::spin(swarm_node);
	rclcpp::shutdown();
	return 0;
}