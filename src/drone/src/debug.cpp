#include <vector>
using std::vector;
#include <string>
using std::string;

#include <memory>
#include <map>
using std::map;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "drone/msg/debug.hpp"

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

class debugNode: public rclcpp::Node
{
public:
	debugNode() : Node("debugNode")
	{
		m_DebugMessageSubscriber = this->create_subscription<drone::msg::Debug>("/debug", 10,
			[this](const drone::msg::Debug::SharedPtr msg) -> void {
				m_DebugMessageMap[msg->id] = *msg;
			}
		);

		m_drawArrowPublisher =
			this->create_publisher<geometry_msgs::msg::Pose>("/drawArrows", 10);
		m_drawRingPublisher =
			this->create_publisher<geometry_msgs::msg::Pose>("/drawRings", 10);

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
		// 收集当前所有quality值
		std::set<double> current_qualities;
		for (const auto& [id, debugMessage] : m_DebugMessageMap) {
			current_qualities.insert(debugMessage.quality);
		}

		// 删除不再存在的quality映射
		for (auto it = m_color_index.begin(); it != m_color_index.end();) {
			if (current_qualities.find(it->first) == current_qualities.end()) {
				it = m_color_index.erase(it);
			} else {
				++it;
			}
		}

		// 为新的quality分配color index
		for (const double& quality : current_qualities) {
			if (m_color_index.find(quality) == m_color_index.end()) {
				uint16_t new_index = 0;
				while (true) {
					bool index_used = false;
					for (const auto& [_, idx] : m_color_index) {
						if (idx == new_index) {
							index_used = true;
							break;
						}
					}
					if (!index_used) break;
					++new_index;
				}
				m_color_index[quality] = new_index;
			}
		}
		
		//如果m_color_index中有大于4的值,而且里面值的总种类少于5种，则把其中大与4的值改成一个空闲的小于4的值
		if (m_color_index.size() <= 5) {
			for (auto [quality, color] : m_color_index) {
				if (color > 4) {
					for (uint16_t i = 0; i < 5; ++i) {
						bool used = false;
						for (const auto [_, color2] : m_color_index) {
							if (color2 == i) {
								used = true;
								break;
							}
						}
						if (used == false) {
							m_color_index[quality] = i;
							break;
						}
					}
				}
			}
		}

		// 输出调试信息
		RCLCPP_INFO(this->get_logger(), "-----------------------");
		for (const auto& [quality, color] : m_color_index) {
			RCLCPP_INFO(this->get_logger(), "   %f %d", quality, color);
		}

		// 绘制ring
		for (const auto& [id, debugMessage] : m_DebugMessageMap) {
			drawRing(
				CVector3(
					debugMessage.middle.x,
					debugMessage.middle.y,
					debugMessage.middle.z + 0.3
				),
				0.2,
				m_color_index[debugMessage.quality]
			);
		}
		//drawArrow(CVector3(), arrow.arrow, arrow.color);
	}

	void drawArrow(CVector3 from, CVector3 to, uint16_t color) {
		geometry_msgs::msg::Pose arrow;
		arrow.position.x = from.GetX();
		arrow.position.y = from.GetY();
		arrow.position.z = from.GetZ();
		arrow.orientation.x = to.GetX();
		arrow.orientation.y = to.GetY();
		arrow.orientation.z = to.GetZ();
		arrow.orientation.w = color;

		m_drawArrowPublisher->publish(arrow);
	}

	void drawRing(CVector3 middle, double radius, uint16_t color) {
		geometry_msgs::msg::Pose ring;
		ring.position.x = middle.GetX();
		ring.position.y = middle.GetY();
		ring.position.z = middle.GetZ();
		ring.orientation.x = radius;
		ring.orientation.w = color;

		m_drawRingPublisher->publish(ring);
	}

private:
	rclcpp::Subscription<drone::msg::Debug>::SharedPtr m_DebugMessageSubscriber;

	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_drawArrowPublisher;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_drawRingPublisher;

	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_SimuTickSubscriber;

	std::map<string, drone::msg::Debug> m_DebugMessageMap;
	map<double, uint16_t> m_color_index;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto debug_node = std::make_shared<debugNode>();
	rclcpp::spin(debug_node);
	rclcpp::shutdown();
	return 0;
}