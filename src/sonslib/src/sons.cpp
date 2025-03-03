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
		std::ostringstream log; // 使用ostringstream
		log << "----------- I am " << m_Data.myId << " Quality " << m_Data.sonsQuality << " ----------------------------" << endl; // 记录信息

		UpdateNeighbors(perceivedNeighbors, time);
		connector.Step(m_Data, time, log);

		// -----------------------------------------------------------
		map<string, CVector3> positions;
		map<string, CVector3> velocities;

		// received message to velocities
		for (const auto& message : receivedMessages) {
			int index = 0;
			double quality = m_Messager.parseDouble(message.binary, index); // 假设从索引0开始解析
			string itsId = m_Messager.parseString(message.binary, index);
			CVector3 velocity = m_Messager.parseCVector3(message.binary, index); // 假设从索引0开始解析

			velocities[itsId] = velocity;
		}

		for (const auto& robot : perceivedNeighbors) {
			positions[robot.id] = robot.GetPosition();
		}

		const int k = 5; // Define the number of nearest neighbors to consider
		std::vector<std::pair<double, string>> distances; // Pair of distance and robot pointer

		for (const SoNSRobot& robot : perceivedNeighbors) {
			if (robot.GetPosition().Length() != 0) {
				double distance = robot.GetPosition().Length();
				distances.emplace_back(distance, robot.id);
			}
		}

		// Sort by distance
		std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
			return a.first < b.first;
		});

		CVector3 vTotal = CVector3();
		int n = 0;
		// Process only the k nearest neighbors
		for (int i = 0; i < std::min(k, static_cast<int>(distances.size())); ++i) {
			if (velocities.find(distances[i].second) != velocities.end()) {
				vTotal += velocities[distances[i].second];
				n++;
			}
		}
		if (n != 0) vTotal = vTotal * (1.0/n) * 0.99;

		double targetDistance = 3;
		// Process only the k nearest neighbors
		for (int i = 0; i < std::min(k, static_cast<int>(distances.size())); ++i) {
			string id = distances[i].second;
			CVector3 v = CVector3(positions[id]).Normalize() * (positions[id].Length() - targetDistance) * 0.06;
			if (v.Length() > 0.5) v = v.Normalize() * 0.5;
			vTotal += v;
		}

		struct SoNSMessage msg;
		msg.id = SoNSBroadCastString;
		m_Messager.pushDouble(msg.binary, m_Data.sonsQuality);
		m_Messager.pushString(msg.binary, m_Data.myId);
		m_Messager.pushCVector3(msg.binary, vTotal);

		struct SoNSStepResult res;
		res.outputVelocity = vTotal;
		res.messages.push_back(msg);
		res.log = log.str(); // 将ostringstream转换为string

		for (int i = 0; i < std::min(k, static_cast<int>(distances.size())); ++i) {
			res.drawArrows.emplace_back(SoNSArrow::Color::RED, positions[distances[i].second]);
		}
		res.drawArrows.emplace_back(SoNSArrow::Color::BLUE, vTotal);

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