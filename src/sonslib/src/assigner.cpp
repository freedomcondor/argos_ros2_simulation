#include "assigner.h"

#include "messager.h"
#include "sons.h"

#include "scaleManager.h"  // for scale add and sub

#include <iostream>
using std::endl;
#include <string>
using std::string;

namespace SoNSLib {

	void SoNSAssigner::Assign(string _child_id, string _to_id) {
		if (sons_->ExistsInChildren(_child_id)) {
			vector<uint8_t> content;
			sons_->messager_.pushString(content, _to_id);
			sons_->messager_.sendCommand(_child_id, CMessager::CommandType::ASSIGN, content);
		}
	}

	void SoNSAssigner::AddParent(string _id) {
		sons_->assignTo_str_ = "";
	}

	void SoNSAssigner::AddChild(string _id) {
		if (sons_->assignTo_str_ == _id) {
			sons_->assignTo_str_ = "";
		}
	}

	void SoNSAssigner::Remove(string _id) {
		if (sons_->ExistsInParent(_id)) {
			sons_->assignTo_str_ = "";
		}
	}

	void SoNSAssigner::PreStep() {
		for (auto& [id, robotp] : sons_->GetTopologicalNeighbors()) {
			robotp->assignScaleOffset.clear();
		}
	}

	void SoNSAssigner::Step(double time) {
		// check assign
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::ASSIGN]) {
			string fromId = command.id;
			if (sons_->ExistsInParent(fromId)) {
				sons_->assignTo_str_ = CMessager::parseString(command.binary);
			}
		}
		// check for recruit
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::RECRUIT]) {
			string fromId = command.id;
			uint index = 0;
			string his_sonsId;
			double his_sonsQuality;
			parseRecruitMessage(command.binary, index, his_sonsId, his_sonsQuality);
			if ((sons_->ExistsInNeighbors(fromId)) &&
			    (his_sonsId == sons_->sonsId_str_) &&
				(fromId == sons_->assignTo_str_)
		       ) {
				string oldParent;
				auto oldScale = sons_->parent_RobotP_->scale;
				// good bye to my parent
				if (sons_->parent_RobotP_ != nullptr) {
					oldParent = sons_->parent_RobotP_->id;
					sons_->messager_.sendCommand(sons_->parent_RobotP_->id, CMessager::CommandType::ASSIGN_BREAK, generateAssignSpecialMessage(fromId, sons_->scale_));
					sons_->Remove(sons_->parent_RobotP_->id);
				}
				// add new parent
				sons_->AddParent(fromId);
				sons_->parent_RobotP_->heartbeatCD = sons_->parameters_.heartbeatCDTime;
				sons_->parent_RobotP_->scale = oldScale;
				sons_->messager_.sendCommand(
					fromId,
					CMessager::CommandType::ACKNOWLEDGEMENT, {}
				);
				sons_->messager_.sendCommand(
					fromId,
					CMessager::CommandType::ASSIGN_ACKNOWLEDGEMENT,
					generateAssignSpecialMessage(oldParent, sons_->scale_)
				);
				//
				// clear my assign
				sons_->assignTo_str_ = "";
			}
		}
		// check for assign ack
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::ASSIGN_ACKNOWLEDGEMENT]) {
			//normal ack is also send, and handled in connector
			string oldParent;
			map<string, uint16_t> incomingScale;
			parseAssignSpecialMessage(command.binary, oldParent, incomingScale);
			/*
			if (sons_->ExistsInTopologicalNeighbors(oldParent)) {
				SoNSScaleManager::SubScaleBfromA(
					sons_->GetTopologicalNeighbors()[oldParent]->assignScaleOffset,
					incomingScale
				);
			}
			*/
			if (sons_->ExistsInChildren(oldParent)) {
				SoNSScaleManager::SubScaleBfromA(
					sons_->children_mapRobotP_[oldParent]->assignScaleOffset,
					incomingScale
				);
			}
		}
		// check for assign break
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::ASSIGN_BREAK]) {
			sons_->Remove(command.id);
			string newParent;
			map<string, uint16_t> incomingScale;
			parseAssignSpecialMessage(command.binary, newParent, incomingScale);
			/*
			if (sons_->ExistsInTopologicalNeighbors(newParent)) {
				SoNSScaleManager::AddScaleBtoA(
					sons_->GetTopologicalNeighbors()[newParent]->assignScaleOffset,
					incomingScale
				);
			}
			*/
			if (sons_->ExistsInChildren(newParent)) {
				SoNSScaleManager::AddScaleBtoA(
					sons_->children_mapRobotP_[newParent]->assignScaleOffset,
					incomingScale
				);
			}
		}
	}

	void SoNSAssigner::parseRecruitMessage(const vector<uint8_t>& _binary, uint& i, string& _his_id, double& _his_quality) {
		_his_id = CMessager::parseString(_binary, i);
		_his_quality = CMessager::parseDouble(_binary, i);
	}

	vector<uint8_t> SoNSAssigner::generateAssignSpecialMessage(const string& _id, const map<string, uint16_t>& _scale) {
		vector<uint8_t> binary;
		CMessager::pushString(binary, _id);
		CMessager::pushIdNumberMap(binary, _scale);
		return binary;
	}
	void SoNSAssigner::parseAssignSpecialMessage(const vector<uint8_t>& _binary, string& _id, map<string, uint16_t>& _scale) {
		uint i = 0;
		_id = CMessager::parseString(_binary, i);
		_scale = CMessager::parseIdNumberMap(_binary, i);
	}
}
