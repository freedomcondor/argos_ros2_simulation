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
      m_debugActuatorSubscriber = m_LoopFunctionRos2NodeHandle->create_subscription<geometry_msgs::msg::Pose>(
         "drawArrows", 10, 
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
      rclcpp::spin_some(m_LoopFunctionRos2NodeHandle); // 处理回调
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::PostStep() {

   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::Destroy() {
      GetSpace().RemoveEntity(*m_pDebugEntity);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::debugActuatorCallback(const geometry_msgs::msg::Pose::SharedPtr pose) {
      // 处理接收到的 pose 消息
      RCLCPP_INFO(m_LoopFunctionRos2NodeHandle->get_logger(), "Received Pose: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
                  pose->position.x, pose->position.y, pose->position.z,
                  pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
   }

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CMyLoopFunctions, "my_loop_functions");
}
