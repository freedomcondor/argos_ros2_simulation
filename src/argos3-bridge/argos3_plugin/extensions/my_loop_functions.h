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
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"

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
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_debugDrawArrowSubscriber;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_debugDrawRingSubscriber;
      void debugDrawArrowCallback(const geometry_msgs::msg::Pose::SharedPtr pose);
      void debugDrawRingCallback(const geometry_msgs::msg::Pose::SharedPtr pose);

      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_pSimuTick;
   };
}

#endif

