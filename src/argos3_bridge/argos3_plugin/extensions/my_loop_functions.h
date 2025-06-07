#ifndef MY_LOOP_FUNCTIONS_H
#define MY_LOOP_FUNCTIONS_H

namespace argos {
   class CDebugEntity;
}

#include <fstream>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>
#include "debug/debug_entity.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "argos3_bridge/msg/tick.hpp"
#include "argos3_bridge/msg/arrows.hpp"
#include "argos3_bridge/msg/rings.hpp"

namespace argos {

   class CMyLoopFunctions : public CLoopFunctions {
   public:

      CMyLoopFunctions() : m_LoopFunctionRos2NodeHandle(nullptr) {}

      virtual ~CMyLoopFunctions() {}

      virtual void Init(TConfigurationNode& t_tree) override;

      virtual void PreStep() override;

      virtual void PostStep() override;

      virtual void Destroy() override;

   private:
      CDebugEntity* m_pDebugEntity;
      rclcpp::Node::SharedPtr m_LoopFunctionRos2NodeHandle;
      rclcpp::Subscription<argos3_bridge::msg::Arrows>::SharedPtr m_debugDrawArrowsSubscriber;
      rclcpp::Subscription<argos3_bridge::msg::Rings>::SharedPtr m_debugDrawRingsSubscriber;
      void debugDrawArrowsCallback(const argos3_bridge::msg::Arrows::SharedPtr msg);
      void debugDrawRingsCallback(const argos3_bridge::msg::Rings::SharedPtr msg);

      rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_pSimuTick;
      rclcpp::Subscription<argos3_bridge::msg::Tick>::SharedPtr m_pSimuTickSubscriber;

      std::map<std::string, uint32_t> m_mapTickIndex;
      uint32_t m_unTickCount = 0;
      bool m_bMoreIncomingMessages = false;
   };
}

#endif

