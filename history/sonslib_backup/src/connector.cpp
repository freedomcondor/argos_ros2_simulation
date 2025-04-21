#include "connector.h"
#include "messager.h"

#include <random>

#include <iostream>
using std::endl;

namespace SoNSLib {

	void SoNSConnector::Initialize(SoNSData& sonsData) {
		sonsData.sonsId = sonsData.myId; // 生成随机数并赋值给sonsQuality
		// 随机数生成器
		std::random_device rd; // 获取随机数种子
		std::mt19937 gen(rd()); // 使用梅森旋转算法生成随机数
		std::uniform_real_distribution<> dis(0.0, 1.0); // 定义范围为0到1的均匀分布
		sonsData.sonsQuality = dis(gen); // 生成随机数并赋值给sonsQuality
	}

	void SoNSConnector::Step(SoNSData& sonsData, double time, std::ostringstream& log) {
		UpdateWaitingList(time);
		// check break
		for (const auto& command : sonsData.GetReceivedCommands()[CMessager::CommandType::BREAK]) {
			string fromId = command.id;
			Remove(sonsData, fromId);
		}

		// check ack
		for (const auto& command : sonsData.GetReceivedCommands()[CMessager::CommandType::ACKNOWLEDGEMENT]) {
			string fromId = command.id;
			if ((sonsData.neighbors.find(fromId) != sonsData.neighbors.end()) &&
			    (m_WaitingList.find(fromId) != m_WaitingList.end())
			   ) {
				sonsData.children[fromId] = &sonsData.neighbors[fromId];
				m_WaitingList.erase(fromId);
			}
		}

		// check recruit messages
		for (const auto& command : sonsData.GetReceivedCommands()[CMessager::CommandType::RECRUIT]) {
			string fromId = command.id;
			uint index = 0;
			string his_sonsId;
			double his_sonsQuality;
			parseRecruitMessage(command.binary, index, his_sonsId, his_sonsQuality);
			if ((sonsData.neighbors.find(fromId) != sonsData.neighbors.end()) &&
			    (his_sonsId != sonsData.sonsId) &&
			    (his_sonsQuality > sonsData.sonsQuality)
			   ) {
				// remove in wait list
				if (m_WaitingList.find(fromId) != m_WaitingList.end()) {m_WaitingList.erase(fromId);};
				// if already has parent
				if (sonsData.parent != nullptr) {
					sonsData.sonsMessager.sendCommand(sonsData.parent->id, CMessager::CommandType::BREAK, {});
					Remove(sonsData, sonsData.parent->id);
				}
				sonsData.parent = &sonsData.neighbors[fromId];
				sonsData.sonsMessager.sendCommand(
					fromId,
					CMessager::CommandType::ACKNOWLEDGEMENT, {}
				);
				sonsData.sonsId = his_sonsId;
				sonsData.sonsQuality = his_sonsQuality;
				UpdateSoNSID(sonsData);
			}
		}

		// recruit all
		for (auto& pair : sonsData.neighbors) {
			if ((m_WaitingList.find(pair.first) == m_WaitingList.end()) &&
			    (!sonsData.ExistsInChildren(pair.first)) &&
			    (!sonsData.ExistsInParent(pair.first))
			   ){
				// Recruit the robot
				Recruit(sonsData, pair.first);
			}
		}

		log << "waiting list: ------------" << endl;
		for (const auto& pair : m_WaitingList) {
			log << "    Robot ID: " << pair.first << ", Waiting Time Countdown: " << pair.second.waitingTimeCountDown << endl;
		}
	};

	void SoNSConnector::UpdateWaitingList(double time) {
		for (auto& pair : m_WaitingList) {
			WaitingSoNSRobot& waitingRobot = pair.second;
			waitingRobot.waitingTimeCountDown -= time;
			if (waitingRobot.waitingTimeCountDown < 0)
				m_WaitingList.erase(pair.first);
		}
	}

	void SoNSConnector::Recruit(SoNSData& sonsData, string id) {
		WaitingSoNSRobot waitingSoNSRobot;
		waitingSoNSRobot.pRobot = &(sonsData.neighbors[id]);
		waitingSoNSRobot.waitingTimeCountDown = sonsData.parameters.recruitWaitingTime;
		m_WaitingList[id] = waitingSoNSRobot;

		sonsData.sonsMessager.sendCommand(
			id,
			CMessager::CommandType::RECRUIT,
			generateRecruitMessage(
				sonsData.sonsId,
				sonsData.sonsQuality
			)
		);
	}

	void SoNSConnector::Remove(SoNSData& sonsData, string _id) {
		if (sonsData.ExistsInChildren(_id)) {sonsData.children.erase(_id);}
		if (sonsData.ExistsInParent(_id)) {
			sonsData.parent = nullptr;
			Initialize(sonsData);
			UpdateSoNSID(sonsData);
		}
	}

	void SoNSConnector::UpdateSoNSID(SoNSData& sonsData) {
		for (auto& pair : sonsData.children) {
			sonsData.sonsMessager.sendCommand(
				pair.first,
				CMessager::CommandType::UPDATE,
				generateRecruitMessage(
					sonsData.sonsId,
					sonsData.sonsQuality
				)
			);
		}
	}

	//------------------------------------------------------
	vector<uint8_t> SoNSConnector::generateRecruitMessage(string sons_id, double sons_quality) {
		vector<uint8_t> content;
		CMessager::pushString(content, sons_id);
		CMessager::pushDouble(content, sons_quality);
		return content;
	}

	void SoNSConnector::parseRecruitMessage(const vector<uint8_t>& _binary, uint& i, string& _his_id, double& _his_quality) {
		_his_id = CMessager::parseString(_binary, i);
		_his_quality = CMessager::parseDouble(_binary, i);
	}
}
