/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

#include "drone_ros2_bridge_controller.h"
#include "geometry_msgs/msg/pose.hpp"

namespace argos {

	/****************************************/
	/****************************************/

	void CTestController::Init(TConfigurationNode& t_tree) {
		//Get flight system actuator / sensor
		m_pcFlightSystemActuator = GetActuator<CCI_DroneFlightSystemActuator>("drone_flight_system");
		m_pcFlightSystemSensor = GetSensor<CCI_DroneFlightSystemSensor>("drone_flight_system");

		// init ROS2
		if (!rclcpp::ok()) { // init only once
			rclcpp::init(0, nullptr);
		}
		// create node
		m_pRos2Node = std::make_shared<rclcpp::Node>("argos3_drone_controller_" + GetId());
		// register publisher
		m_pPoseSensorPublisher = m_pRos2Node->create_publisher<geometry_msgs::msg::Pose>("drone_topic_" + GetId(), 10);
	}

	/****************************************/
	/****************************************/

	void CTestController::ControlStep() {
		// read pose readings and publish to poseSensor topic
		CVector3 currentPosition = m_pcFlightSystemSensor->GetPosition();
		CVector3 currentOrientationInEuler = m_pcFlightSystemSensor->GetOrientation();
		CQuaternion currentOrientation;
		currentOrientation.FromEulerAngles(
			CRadians(currentOrientationInEuler.GetZ()),
			CRadians(currentOrientationInEuler.GetY()),
			CRadians(currentOrientationInEuler.GetX())
		);

		geometry_msgs::msg::Pose poseMessage;
		poseMessage.position.x = currentPosition.GetX();
		poseMessage.position.y = currentPosition.GetY();
		poseMessage.position.z = currentPosition.GetZ();

		poseMessage.orientation.x = currentOrientation.GetX();
		poseMessage.orientation.y = currentOrientation.GetY();
		poseMessage.orientation.z = currentOrientation.GetZ();
		poseMessage.orientation.w = currentOrientation.GetW();

		m_pPoseSensorPublisher->publish(poseMessage);

		// spin ros node
		rclcpp::spin_some(m_pRos2Node);
	}

	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(CTestController, "test_controller");

}



