#include "connector.h"
#include "messager.h"
#include "sons.h"

#include <random>

#include <iostream>
using std::endl;

namespace SoNSLib {

	void SoNSConnector::Init() {
		sons_->sonsId_str_ = sons_->myId_str_;
		// 随机数生成器
		std::random_device rd; // 获取随机数种子
		std::mt19937 gen(rd()); // 使用梅森旋转算法生成随机数
		std::uniform_real_distribution<> dis(0.0, 1.0); // 定义范围为0到1的均匀分布

		sons_->sonsQuality_f_= dis(gen); // 生成随机数并赋值给sonsQuality
	}

	void SoNSConnector::Step(double time) {
		sons_->log_ << "-- recruitor -----------------------" << endl;
		sons_->log_ << "\tlockCD = " << lockCD << std::endl;
		if (lockCD >= 0) lockCD -= time;
		UpdateWaitingList(time);

		// check heartbeat
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::HEARTBEAT]) {
			string fromId = command.id;
			if (sons_->ExistsInParent(fromId)) {
				sons_->parent_RobotP_->heartbeatCD = sons_->parameters_.heartbeatCDTime;
			} else if (sons_->ExistsInChildren(fromId)) {
				sons_->children_mapRobotP_[fromId]->heartbeatCD = sons_->parameters_.heartbeatCDTime;
			}
		}

		// countdown heartbeat for parent
		if (sons_->parent_RobotP_ != nullptr) {
			sons_->parent_RobotP_->heartbeatCD -= time;
			if (sons_->parent_RobotP_->heartbeatCD < 0) {
				// Send BREAK message to parent
				sons_->messager_.sendCommand(
					sons_->parent_RobotP_->id,
					CMessager::CommandType::BREAK,
					{}
				);
				sons_->RemoveWithUpdate(sons_->parent_RobotP_->id);
			}
		}

		// countdown heartbeat for children
		for (auto it = sons_->children_mapRobotP_.begin(); it != sons_->children_mapRobotP_.end();) {
			it->second->heartbeatCD -= time;
			if (it->second->heartbeatCD < 0) {
				// Send BREAK message to the child
				sons_->messager_.sendCommand(
					it->second->id,
					CMessager::CommandType::BREAK,
					{}
				);
				it = sons_->children_mapRobotP_.erase(it);
			} else {
				++it;
			}
		}

		// check ack
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::ACKNOWLEDGEMENT]) {
			string fromId = command.id;
			if ((sons_->ExistsInNeighbors(fromId)) &&
			    (m_WaitingList.find(fromId) != m_WaitingList.end())
			   ) {
				sons_->children_mapRobotP_[fromId] = &sons_->neighbors_mapRobot_[fromId];
				sons_->children_mapRobotP_[fromId]->heartbeatCD = sons_->parameters_.heartbeatCDTime;
				m_WaitingList.erase(fromId);
			}
		}

		// check break
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::BREAK]) {
			string fromId = command.id;
			sons_->RemoveWithUpdate(fromId);
		}

		// check update
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::UPDATE]) {
			string fromId = command.id;
			if (sons_->ExistsInParent(fromId)) {
				uint index = 0;
				string his_sonsId;
				double his_sonsQuality;
				parseRecruitMessage(command.binary, index, his_sonsId, his_sonsQuality);
				// prevent loop
				if (his_sonsId == sons_->myId_str_) {
					sons_->RemoveWithUpdate(fromId);
				}
				else {
					sons_->sonsId_str_ = his_sonsId;
					sons_->sonsQuality_f_ = his_sonsQuality;
					UpdateSoNSID();
				}
			}
		}

		// check recruit messages
		string bestFromId;
		string bestSonsId;
		double bestSonsQuality = sons_->sonsQuality_f_;
		bool foundBetter = false;

		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::RECRUIT]) {
			string fromId = command.id;
			uint index = 0;
			string his_sonsId;
			double his_sonsQuality;
			parseRecruitMessage(command.binary, index, his_sonsId, his_sonsQuality);
			if ((sons_->ExistsInNeighbors(fromId)) &&
			    (his_sonsId != sons_->sonsId_str_) &&
			    (his_sonsQuality > bestSonsQuality) &&
			    (lockCD < 0)
			   ) {
				bestFromId = fromId;
				bestSonsId = his_sonsId;
				bestSonsQuality = his_sonsQuality;
				foundBetter = true;
			}
		}

		if (foundBetter) {
			// remove in wait list
			if (m_WaitingList.find(bestFromId) != m_WaitingList.end()) {m_WaitingList.erase(bestFromId);};
			// if already has parent
			if (sons_->parent_RobotP_ != nullptr) {
				sons_->messager_.sendCommand(sons_->parent_RobotP_->id, CMessager::CommandType::BREAK, {});
				sons_->Remove(sons_->parent_RobotP_->id);
			}
			sons_->parent_RobotP_ = &sons_->neighbors_mapRobot_[bestFromId];
			sons_->parent_RobotP_->heartbeatCD = sons_->parameters_.heartbeatCDTime;
			sons_->messager_.sendCommand(
				bestFromId,
				CMessager::CommandType::ACKNOWLEDGEMENT, {}
			);
			sons_->sonsId_str_ = bestSonsId;
			sons_->sonsQuality_f_ = bestSonsQuality;
			UpdateSoNSID();
		}

		// Send heartbeat to parent and all children
		if (sons_->parent_RobotP_ != nullptr) {
			sons_->messager_.sendCommand(
				sons_->parent_RobotP_->id,
				CMessager::CommandType::HEARTBEAT,
				{}
			);
		}
		for (auto& pair : sons_->children_mapRobotP_) {
			sons_->messager_.sendCommand(
				pair.first,
				CMessager::CommandType::HEARTBEAT,
				{}
			);
		}
		// recruit all
		for (auto& pair : sons_->neighbors_mapRobot_) {
			if ((m_WaitingList.find(pair.first) == m_WaitingList.end()) &&
			    (!sons_->ExistsInChildren(pair.first)) &&
			    (!sons_->ExistsInParent(pair.first))
			   ){
				// Recruit the robot
				sons_->Recruit(pair.first);
			}
		}
	};

	void SoNSConnector::UpdateWaitingList(double time) {
		// remove outdated robots
		for (auto it = m_WaitingList.begin(); it != m_WaitingList.end();) {
			it->second.waitingTimeCountDown -= time;
			if (it->second.waitingTimeCountDown < 0) {
				it = m_WaitingList.erase(it);
			}
			else {
				it++;
			}
		}
	}

	void SoNSConnector::Recruit(string id) {
		WaitingSoNSRobot waitingSoNSRobot;
		waitingSoNSRobot.pRobot = &(sons_->neighbors_mapRobot_[id]);
		waitingSoNSRobot.waitingTimeCountDown = sons_->parameters_.recruitWaitingTime;
		m_WaitingList[id] = waitingSoNSRobot;

		sons_->messager_.sendCommand(
			id,
			CMessager::CommandType::RECRUIT,
			generateRecruitMessage(
				sons_->sonsId_str_,
				sons_->sonsQuality_f_
			)
		);
	}

	void SoNSConnector::Remove(string _id) {
		if (sons_->ExistsInChildren(_id)) {sons_->children_mapRobotP_.erase(_id);}
		if (sons_->ExistsInParent(_id)) {
			sons_->parent_RobotP_ = nullptr;
		}
	}

	void SoNSConnector::RemoveWithUpdate(string _id) {
		if (sons_->ExistsInParent(_id)) {
			sons_->Remove(_id);
			Init();
			UpdateSoNSID();
		}
		else {
			sons_->Remove(_id);
		}
	}

	void SoNSConnector::UpdateSoNSID() {
		lockCD = (sons_->depth_ + 2) * sons_->step_time_;
		for (auto& pair : sons_->children_mapRobotP_) {
			sons_->messager_.sendCommand(
				pair.first,
				CMessager::CommandType::UPDATE,
				generateRecruitMessage(
					sons_->sonsId_str_,
					sons_->sonsQuality_f_
				)
			);
		}
		for (auto& pair : m_WaitingList) {
			sons_->messager_.sendCommand(
				pair.first,
				CMessager::CommandType::UPDATE,
				generateRecruitMessage(
					sons_->sonsId_str_,
					sons_->sonsQuality_f_
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
