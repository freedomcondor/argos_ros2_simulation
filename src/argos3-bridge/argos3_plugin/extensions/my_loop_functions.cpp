#include "my_loop_functions.h"

namespace argos {

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::Init(TConfigurationNode& t_tree) {
      // ros2
      if (!rclcpp::ok()) { // init only once
         rclcpp::init(0, nullptr);
      }
      m_LoopFunctionRos2NodeHandle = std::make_shared<rclcpp::Node>("argos_loop_function_node");
		m_pSimuTick = m_LoopFunctionRos2NodeHandle->create_publisher<std_msgs::msg::Empty>("/simuTick", 10);
      m_debugActuatorSubscriber = m_LoopFunctionRos2NodeHandle->create_subscription<geometry_msgs::msg::Pose>(
         "/drawArrows", 1000,
         std::bind(&CMyLoopFunctions::debugActuatorCallback, this, std::placeholders::_1)
      );

      /* debug entity for arrow draw */
      m_pDebugEntity = new CDebugEntity(nullptr, "DebugEntityInLoopFunction");
      GetSpace().AddEntity(*m_pDebugEntity);

      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(2,0,0), CColor::RED);
      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(0,1,0), CColor::GREEN);
      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(0,0,1), CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::PreStep() {
      m_pDebugEntity->GetArrows().clear();

      std_msgs::msg::Empty msg;
      m_pSimuTick->publish(msg);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::PostStep() {
      for (int i = 0; i < 1000; i++)
         rclcpp::spin_some(m_LoopFunctionRos2NodeHandle);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::Destroy() {
      GetSpace().RemoveEntity(*m_pDebugEntity);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::debugActuatorCallback(const geometry_msgs::msg::Pose::SharedPtr pose) {
      CVector3 from = CVector3(
         pose->position.x,
         pose->position.y,
         pose->position.z
      );
      CVector3 to = CVector3(
         pose->orientation.x,
         pose->orientation.y,
         pose->orientation.z
      );

      CColor color;
      switch ((int)pose->orientation.w) {
         case 0:  color = CColor::RED;    break;
         case 1:  color = CColor::GREEN;  break;
         case 2:  color = CColor::BLUE;   break;
         case 3:  color = CColor::YELLOW; break;
         case 4:  color = CColor::BLACK;  break;
         default: color = CColor::WHITE; break;
      }

      m_pDebugEntity->GetArrows().emplace_back(from, to, color);
   }

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CMyLoopFunctions, "my_loop_functions");
}
