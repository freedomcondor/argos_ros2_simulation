#include "sons.h"

#include <iostream>
#include <random>
#include <sstream>
using std::cout;
using std::endl;
#include <algorithm>
#include <map>
using std::map;

namespace SoNSLib {
	SoNS::SoNS() {

	}

	void SoNS::SetId(string _myId, string _myType) {
		m_Data.myId = _myId;
		m_Data.myType = _myType;

		// 随机数生成器
		std::random_device rd; // 获取随机数种子
		std::mt19937 gen(rd()); // 使用梅森旋转算法生成随机数
		std::uniform_real_distribution<> dis(0.0, 1.0); // 定义范围为0到1的均匀分布

		m_Data.sonsQuality = dis(gen); // 生成随机数并赋值给sonsQuality
	}

	struct SoNSStepResult SoNS::Step(double time, const vector<SoNSRobot>& perceivedNeighbors, const vector<struct SoNSMessage>& receivedMessages) {
		m_Data.messagesToSend.clear();

		std::ostringstream log; // 使用ostringstream
		log << "----------- I am " << m_Data.myId << " ------------------------------------" << endl; // 记录信息

		for (const auto& message : receivedMessages) {
			int index = 0;
			log << "Received message: " << message.id << " " << SoNSMessager::parseString(message.binary, index) << endl; // 打印接收到的消息内容
		}

		UpdateNeighbors(perceivedNeighbors, time);
		connector.Step(m_Data, time, log);

		// -----------------------------------------------------------
		struct SoNSStepResult res;
		res.outputVelocity = CVector3();
		for (auto& pair : m_Data.messagesToSend) res.messages.push_back(pair.second);
		res.log = log.str(); // 将ostringstream转换为string

		//res.drawArrows.emplace_back(SoNSArrow::Color::BLUE, vTotal);

		return res;
	}

	void SoNS::UpdateNeighbors(const vector<SoNSRobot>& perceivedNeighbors, double time) {
		for (const SoNSRobot& robot : perceivedNeighbors) {
			if (m_Data.neighbors.find(robot.id) == m_Data.neighbors.end()) {
				m_Data.neighbors[robot.id] = robot; // If it doesn't exist, add the robot
				m_Data.neighbors[robot.id].heartbeat = m_Data.parameters.heartbeatTime;
			}
			else {
				m_Data.neighbors[robot.id].transform = robot.transform; // Update the transform of the existing robot
				m_Data.neighbors[robot.id].heartbeat = m_Data.parameters.heartbeatTime;
			}
		}

		for (auto& pair : m_Data.neighbors) {
			pair.second.heartbeat -= time; // Decrease the heartbeat by the elapsed time
			if (pair.second.heartbeat < 0) {
				// TODO
				//connector.remove();
			}
		}
	}
}