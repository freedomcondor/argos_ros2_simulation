#include <vector>
using std::vector;
#include <string>
using std::string;

#include <memory>
#include <map>
using std::map;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "drone/msg/pose_sharing.hpp"
#include "drone/msg/message.hpp"
#include "drone/msg/debug.hpp"

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

#include "sonslib/sons.h"
using namespace SoNSLib;

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

		sons.Init(m_strMyID, "drone");

		m_VelocityActuatorPublisher =
			this->create_publisher<geometry_msgs::msg::Pose>("droneVelocityActuator", 10);

		// receive my pose, and publish it into position sharing with my ID
		m_PositionSensorSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("dronePoseSensor", 10,
			[this](geometry_msgs::msg::Pose::UniquePtr msg) -> void {
				m_CurrentTransform = createCTransformFromGeometryMsgPose(*msg);
			}
		);

		m_SwarmCommunicationSubscriber = this->create_subscription<drone::msg::Message>("communication", 2000,
			[this](const drone::msg::Message::SharedPtr msg) -> void {
				struct SoNSMessage sonsMsg;
				sonsMsg.id = msg->id; // Set the ID from the received message
				std::vector<uint8_t> binaryData(msg->binary.data.begin(), msg->binary.data.end());
				sonsMsg.binary = binaryData; // Copy the binary data directly
				m_receivedMessages.push_back(sonsMsg);
			}
		);

		// declare position sharing publisher and subscriber
		m_PoseSharingPublisher =
			this->create_publisher<drone::msg::PoseSharing>("/poseSharing", 1000);

		this->declare_parameter("distance_threshold", 10.0);
		this->declare_parameter("target_distance", 3.0);

		m_PoseSharingSubscriber = this->create_subscription<drone::msg::PoseSharing>("/poseSharing", 1000,
			[this](const drone::msg::PoseSharing::SharedPtr msg) -> void{
				if (msg->id == m_strMyID) return;

				double distanceThreshold;
				this->get_parameter("distance_threshold", distanceThreshold);

				CTransform neighbour = createCTransformFromGeometryMsgPose(msg->pose);
				if ((neighbour.m_Position - m_CurrentTransform.m_Position).Length() < distanceThreshold) {
					m_SwarmPoses[msg->id] = neighbour - m_CurrentTransform;
					if (m_SwarmCommunicationPublishers.find(msg->id) == m_SwarmCommunicationPublishers.end()) {
						m_SwarmCommunicationPublishers[msg->id] = this->create_publisher<drone::msg::Message>("/" + msg->id + "/communication", 2000);
					}
				}
				else {
					m_SwarmPoses.erase(msg->id); // Remove the drone's pose if it's too far
					m_SwarmCommunicationPublishers.erase(msg->id); // Unregister the publisher if out of range
				}
			}
		);

		m_drawArrowPublisher =
			this->create_publisher<geometry_msgs::msg::Pose>("/drawArrows", 100);
		m_drawRingPublisher =
			this->create_publisher<geometry_msgs::msg::Pose>("/drawRings", 100);
		m_debugPublisher =
			this->create_publisher<drone::msg::Debug>("/debug", 100);

		// Subscribe to simuTick topic to start timer on first signal
		m_SimuTickSubscriber = this->create_subscription<std_msgs::msg::Empty>("/simuTick", 10,
			[this](const std_msgs::msg::Empty::SharedPtr) -> void {
				// Only start timer on first tick if not already started
				// step timer, call step() in a frequency
				/*
				if (!m_Timer) {
					m_Timer = this->create_wall_timer(
						std::chrono::milliseconds(200), // ms
						[this]() { this->step(); }
					);
				}
				*/
				step();
			}
		);
	}

	void step() {
		// share myself
		drone::msg::PoseSharing poseSharingMsg;
		poseSharingMsg.id = m_strMyID;
		poseSharingMsg.pose = createGeometryMsgPoseFromCTransform(m_CurrentTransform);
		m_PoseSharingPublisher->publish(poseSharingMsg);

		// functions -------------------------------------------------------------
		// construct neighbours
		vector<SoNSRobot> neighbours;
		for (const auto& swarmPose : m_SwarmPoses) {
			const string& id = swarmPose.first;
			const CTransform& pose = swarmPose.second;
			neighbours.emplace_back(id, "drone", pose);
		}
		// run step
		struct SoNSStepResult result = sons.Step(0.2, neighbours, m_receivedMessages);
		m_receivedMessages.clear();

		// Log the message from SoNS
		RCLCPP_INFO(this->get_logger(), "%s", result.log.c_str());

		// enforce output velocity
		setVelocity(result.outputVelocity);
		// send result.messages
		for (const auto& message : result.messages) {
			sendMessage(message.id, message.binary);
		}
		// drawArrows
		for (const auto& arrow: result.drawArrows) {
			drawArrow(CVector3(), arrow.arrow, arrow.color);
		}
		// drawRings
		for (const auto& ring: result.drawRings) {
			drawRing(ring.middle, ring.radius, ring.color);
		}
		// Debug Msg
		drone::msg::Debug debug;
		debug.quality = result.debugMessage.quality;
		debug.id = m_strMyID;
		debug.middle.x = m_CurrentTransform.m_Position.GetX();
		debug.middle.y = m_CurrentTransform.m_Position.GetY();
		debug.middle.z = m_CurrentTransform.m_Position.GetZ();
		m_debugPublisher->publish(debug);
	}

	//------------------------------------------------------------------------------
	void sendMessage(const string& id, const string& msg) {
		std::vector<uint8_t> binaryData(msg.begin(), msg.end());
		sendMessage(id, binaryData);
	}

	void sendMessage(const string& id, const vector<uint8_t>& binaryData) {
		drone::msg::Message message;
		message.id = m_strMyID; // 设置ID
		message.binary.data = binaryData; // 将二进制数据赋值给消息

		if (id == "BROADCAST") {
			for (const auto& publisher : m_SwarmCommunicationPublishers) {
				auto publisherPtr = publisher.second;
				publisherPtr->publish(message);
			}
		} else {
			if (m_SwarmCommunicationPublishers.find(id) != m_SwarmCommunicationPublishers.end()) {
				auto publisherPtr = m_SwarmCommunicationPublishers[id];
				publisherPtr->publish(message);
			}
		}
	}

	void drawArrow(CVector3 from, CVector3 to, SoNSArrow::Color color) {
		from = m_CurrentTransform * from;
		to = m_CurrentTransform * to;
		geometry_msgs::msg::Pose arrow;
		arrow.position.x = from.GetX();
		arrow.position.y = from.GetY();
		arrow.position.z = from.GetZ();
		arrow.orientation.x = to.GetX();
		arrow.orientation.y = to.GetY();
		arrow.orientation.z = to.GetZ();

		switch (color) {
			case SoNSArrow::Color::RED:    arrow.orientation.w = 0; break;
			case SoNSArrow::Color::GREEN:  arrow.orientation.w = 1; break;
			case SoNSArrow::Color::BLUE:   arrow.orientation.w = 2; break;
			case SoNSArrow::Color::YELLOW: arrow.orientation.w = 3; break;
			case SoNSArrow::Color::BLACK:  arrow.orientation.w = 4; break;
			default:                       arrow.orientation.w = 5; break;
		}

		m_drawArrowPublisher->publish(arrow);
	}

	void drawRing(CVector3 middle, double radius, SoNSRing::Color color) {
		middle = m_CurrentTransform * middle;
		geometry_msgs::msg::Pose ring;
		ring.position.x = middle.GetX();
		ring.position.y = middle.GetY();
		ring.position.z = middle.GetZ();
		ring.orientation.x = radius;

		switch (color) {
			case SoNSRing::Color::RED:    ring.orientation.w = 0; break;
			case SoNSRing::Color::GREEN:  ring.orientation.w = 1; break;
			case SoNSRing::Color::BLUE:   ring.orientation.w = 2; break;
			case SoNSRing::Color::YELLOW: ring.orientation.w = 3; break;
			case SoNSRing::Color::BLACK:  ring.orientation.w = 4; break;
			default:                      ring.orientation.w = 5; break;
		}

		m_drawRingPublisher->publish(ring);
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

	CTransform createCTransformFromGeometryMsgPose(const geometry_msgs::msg::Pose& _pose) {
		CTransform res;
		res.m_Position = CVector3(
			_pose.position.x,
			_pose.position.y,
			_pose.position.z
		);
		res.m_Orientation = CQuaternion(
			_pose.orientation.w,
			_pose.orientation.x,
			_pose.orientation.y,
			_pose.orientation.z
		);
		return res;
	}

	geometry_msgs::msg::Pose createGeometryMsgPoseFromCTransform(const CTransform& _tf) {
		geometry_msgs::msg::Pose poseMsg;
		poseMsg.position.x = _tf.m_Position.GetX();
		poseMsg.position.y = _tf.m_Position.GetY();
		poseMsg.position.z = _tf.m_Position.GetZ();
		poseMsg.orientation.w = _tf.m_Orientation.GetW();
		poseMsg.orientation.x = _tf.m_Orientation.GetX();
		poseMsg.orientation.y = _tf.m_Orientation.GetY();
		poseMsg.orientation.z = _tf.m_Orientation.GetZ();
		return poseMsg;
	}

private:
	string m_strMyID;
	CTransform m_CurrentTransform;
	std::map<string, CTransform> m_SwarmPoses;

	std::map<string, rclcpp::Publisher<drone::msg::Message>::SharedPtr> m_SwarmCommunicationPublishers;
	rclcpp::Subscription<drone::msg::Message>::SharedPtr m_SwarmCommunicationSubscriber;

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_PositionSensorSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_VelocityActuatorPublisher;

	rclcpp::Subscription<drone::msg::PoseSharing>::SharedPtr m_PoseSharingSubscriber;
	rclcpp::Publisher<drone::msg::PoseSharing>::SharedPtr m_PoseSharingPublisher;

	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_drawArrowPublisher;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_drawRingPublisher;
	rclcpp::Publisher<drone::msg::Debug>::SharedPtr m_debugPublisher;

	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_SimuTickSubscriber;

	rclcpp::TimerBase::SharedPtr m_Timer; // 定时器

	// SoNS
	vector<struct SoNSMessage> m_receivedMessages;
	SoNS sons;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto swarm_node = std::make_shared<droneSwarmSystem>();
	rclcpp::spin(swarm_node);
	rclcpp::shutdown();
	return 0;
}