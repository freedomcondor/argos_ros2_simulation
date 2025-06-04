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
      m_debugDrawArrowSubscriber = m_LoopFunctionRos2NodeHandle->create_subscription<geometry_msgs::msg::Pose>(
         "/drawArrows", 5000,
         std::bind(&CMyLoopFunctions::debugDrawArrowCallback, this, std::placeholders::_1)
      );
      m_debugDrawRingSubscriber = m_LoopFunctionRos2NodeHandle->create_subscription<geometry_msgs::msg::Pose>(
         "/drawRings", 5000,
         std::bind(&CMyLoopFunctions::debugDrawRingCallback, this, std::placeholders::_1)
      );

      /* debug entity for arrow draw */
      m_pDebugEntity = new CDebugEntity(nullptr, "DebugEntityInLoopFunction");
      GetSpace().AddEntity(*m_pDebugEntity);

      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(2,0,0), CColor::RED);
      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(0,1,0), CColor::GREEN);
      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(0,0,1), CColor::BLACK);
      //m_pDebugEntity->GetRings().emplace_back(CVector3(0,0,0.0), 1, CColor::BLACK);

      // get entities for drone tick index
      CEntity::TVector& tRootEntityVector = GetSpace().GetRootEntityVector();
      for(CEntity* pc_entity : tRootEntityVector) {
         // check if pc_entity->GetId() starts with drone
         if (pc_entity->GetId().find("drone") == 0) {
            m_mapTickIndex[pc_entity->GetId()] = m_unTickCount;
         }
      }

      // tick ping pong
		m_pSimuTick = m_LoopFunctionRos2NodeHandle->create_publisher<std_msgs::msg::UInt32>("/simuTick", 10);
      m_pSimuTickSubscriber = m_LoopFunctionRos2NodeHandle->create_subscription<std_msgs::msg::String>(
         "/simuTickBack", 1000,
			[this](std_msgs::msg::String::UniquePtr msg) -> void {
            // parse msg.data, break from "_", the first half is the id, the second half is a uint32_t, is tick number
            std::string id = msg->data.substr(0, msg->data.find("_"));
            std::string number = msg->data.substr(msg->data.find("_")+1);
            m_mapTickIndex[id] = std::stoi(number);
			}
      );
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::PreStep() {
      m_unTickCount++;

      m_pDebugEntity->GetArrows().clear();
      m_pDebugEntity->GetRings().clear();

      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(2,0,0), CColor::RED);
      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(0,1,0), CColor::GREEN);
      m_pDebugEntity->GetArrows().emplace_back(CVector3(0,0,0), CVector3(0,0,1), CColor::BLACK);
      //m_pDebugEntity->GetRings().emplace_back(CVector3(0,0,0.0), 1, CColor::BLACK);

      auto msg = std_msgs::msg::UInt32();
      msg.data = m_unTickCount;
      m_pSimuTick->publish(msg);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::PostStep() {
      // check m_mapTickIndex, if all equal to m_unTickCount, then all entities have finished their step
      bool bAllEntitiesFinished = false;
      bool bAnyEntitiesFinished = false;

      //while (bAllEntitiesFinished == false) {
      while (bAnyEntitiesFinished == false) {

         rclcpp::spin_some(m_LoopFunctionRos2NodeHandle);

         for (auto& [key, value] : m_mapTickIndex) {
            // check all entities have finished their step
            if (value < m_unTickCount) {
               bAllEntitiesFinished = false;
               break;
            }
            else {
               bAllEntitiesFinished = true;
            }
            // check anyone reach
            if (value >= m_unTickCount) {
               bAnyEntitiesFinished = true;
               break;
            }
         }
      }

      for (int i = 0; i < 500; i++)
         rclcpp::spin_some(m_LoopFunctionRos2NodeHandle);
      /*
      m_bMoreIncomingMessages = true;
      while (m_bMoreIncomingMessages == true) {
         m_bMoreIncomingMessages = false;
         for (int i = 0; i < 20; i++)
            rclcpp::spin_some(m_LoopFunctionRos2NodeHandle);
      }
      */
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::Destroy() {
      GetSpace().RemoveEntity(*m_pDebugEntity);
   }

   /****************************************/
   /****************************************/

   void CMyLoopFunctions::debugDrawArrowCallback(const geometry_msgs::msg::Pose::SharedPtr pose) {
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
      m_bMoreIncomingMessages = true;
   }

   void CMyLoopFunctions::debugDrawRingCallback(const geometry_msgs::msg::Pose::SharedPtr pose) {
      CVector3 middle = CVector3(
         pose->position.x,
         pose->position.y,
         pose->position.z
      );
      double radius = pose->orientation.x;

      CColor color;
      switch ((int)pose->orientation.w) {
         case 0:  color = CColor::RED;    break;
         case 1:  color = CColor::GREEN;  break;
         case 2:  color = CColor::BLUE;   break;
         case 3:  color = CColor::YELLOW; break;
         case 4:  color = CColor::BLACK;  break;
         default: color = CColor::WHITE; break;
      }

      m_pDebugEntity->GetRings().emplace_back(middle, radius, color);
      m_bMoreIncomingMessages = true;
   }

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CMyLoopFunctions, "my_loop_functions");
}
