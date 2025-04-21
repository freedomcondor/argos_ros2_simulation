#include "sons.h"

#include <iostream>
#include <sstream>
using std::cout;
using std::endl;
#include <algorithm>
#include <map>
using std::map;

namespace SoNSLib {
	SoNS::SoNS() {

	}

	void SoNS::Initialize(string _myId, string _myType) {
		m_Data.myId = _myId;
		m_Data.myType = _myType;

		sonsConnector.Initialize(m_Data);
	}

	struct SoNSStepResult SoNS::Step(
		double time,
		const vector<SoNSRobot>& perceivedNeighbors,
		const vector<struct SoNSMessage>& receivedMessages
	) {
		std::ostringstream log; // 使用ostringstream
		log << endl;
		log << "----------- I am " << m_Data.myId << ", I belong to " << m_Data.sonsId << ", My quality is " << m_Data.sonsQuality << "------------------------------------" << endl; // 记录信息

		//- debug print raw messages ---------------------------------------------
		for (const auto& message : receivedMessages) {
			log << "Message binary in hex: ";
			for (const uint8_t byte : message.binary) {

				log << std::hex << std::uppercase;
				if (static_cast<int>(byte) < 16) log << "0";
				log << static_cast<int>(byte) << " ";
			}
			log << std::dec << endl;

			uint index = 1; // 0xCC header
			log << "Received message from : " << message.id << " "
				<< CMessager::parseCommandType(message.binary, index) << " "
				<< CMessager::parseUint16(message.binary, index) << " "
				<< CMessager::parseString(message.binary, index) << " "
				<< CMessager::parseDouble(message.binary, index) << " "
				<< endl;
		}
		log << "neighbours : ----------------------" << endl;
		for (auto& pair : m_Data.neighbors) {
			log << pair.first << "\t ";
		}
		log << endl;
		log << "parent : ----------------------" << endl;
		if (m_Data.parent != nullptr) log << m_Data.parent->id;
		log << endl;
		log << "children : ----------------------" << endl;
		for (auto& pair : m_Data.children) {
			log << pair.first << "\t ";
		}
		log << endl;

		//-------------------------------------------------------------------
		m_Data.sonsMessager.clearMessages();
		m_Data.sonsMessager.OrganizeReceivedCommands(receivedMessages);

		UpdateNeighbors(perceivedNeighbors, time);
		log << "after UpdateNeighbors" << endl;
		sonsConnector.Step(m_Data, time, log);
		log << "after connector step" << endl;

		// -----------------------------------------------------------
		struct SoNSStepResult res;
		res.outputVelocity = CVector3();
		map<string, vector<uint8_t>> messageMap = m_Data.sonsMessager.combineCommands();
		for (auto& pair : messageMap) res.messages.push_back({pair.first, pair.second});
		res.log = log.str(); // 将ostringstream转换为string

		for (auto& pair : m_Data.children) {
			res.drawArrows.emplace_back(SoNSArrow::Color::BLUE, pair.second->GetPosition());
		}

		return res;
	}

	void SoNS::UpdateNeighbors(const vector<SoNSRobot>& perceivedNeighbors, double time) {
		for (const SoNSRobot& robot : perceivedNeighbors) {
			if (m_Data.neighbors.find(robot.id) == m_Data.neighbors.end()) {
				m_Data.neighbors[robot.id] = robot; // If it doesn't exist, add the robot
				m_Data.neighbors[robot.id].transform = robot.transform;
				m_Data.neighbors[robot.id].heartbeat = m_Data.parameters.heartbeatTime;
			}
			else {
				m_Data.neighbors[robot.id].transform = robot.transform;
				m_Data.neighbors[robot.id].heartbeat = m_Data.parameters.heartbeatTime;
			}
		}

		for (auto it = m_Data.neighbors.begin(); it != m_Data.neighbors.end();) {
			it->second.heartbeat -= time; // Decrease the heartbeat by the elapsed time
			if (it->second.heartbeat < 0) {
				if (m_Data.ExistsInChildren(it->first) || m_Data.ExistsInParent(it->first))
					m_Data.sonsMessager.sendCommand(it->first, CMessager::CommandType::BREAK, {});
				sonsConnector.Remove(m_Data, it->first);
				it = m_Data.neighbors.erase(it);
			}
			else {
				it++;
			}
		}
	}
}