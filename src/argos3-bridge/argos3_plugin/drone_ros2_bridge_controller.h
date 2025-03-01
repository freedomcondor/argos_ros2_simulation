/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

// include argos ci controller
#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>
#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_sensor.h>

// include ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace argos {

	class CTestController : public CCI_Controller {

	public:

		CTestController() {}

		virtual ~CTestController() {}

		virtual void Init(TConfigurationNode& t_tree);
		virtual void ControlStep();

	private:
		CCI_DroneFlightSystemActuator* m_pcFlightSystemActuator;
		CCI_DroneFlightSystemSensor* m_pcFlightSystemSensor;

		UInt32 m_unStepCount = 0;

		std::shared_ptr<rclcpp::Node> m_pRos2Node;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pPoseSensorPublisher;
		rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_pPoseActuatorSubscriber;
		rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_pVelocityActuatorSubscriber;
	};
}
