#include "sons.h"

#include <iostream>
using std::cout;
using std::endl;
//#include <sstream>
//#include <algorithm>
#include <map>
using std::map;

#include "sons.h"
//#include "ModuleA.h"
//#include "ModuleB.h"

namespace SoNSLib {

	SoNS::SoNS() : sonsConnector(*this) {
		RegisterModule(std::shared_ptr<SoNSConnector>(&sonsConnector, [](SoNSConnector*){}));
	}

	void SoNS::Init(string _myId, string _myType) {
		myId_str_ = _myId;
		myType_str_ = _myType;

		for (auto& module : modules_) {
			module->Init();
		}
	}

	void SoNS::RegisterModule(std::shared_ptr<SoNSModule> module) {
		modules_.push_back(module);
		
		// 可选：保存具体模块的引用
		/*
		if (auto moduleA = std::dynamic_pointer_cast<ModuleA>(module)) {
			moduleA_ = moduleA;
		}
		if (auto moduleB = std::dynamic_pointer_cast<ModuleB>(module)) {
			moduleB_ = moduleB;
		}
		*/
	}

	void SoNS::PreStep() {
		for (auto& module : modules_) {
			module->PreStep();
		}
	}

	void SoNS::PostStep() {
		for (auto& module : modules_) {
			module->PostStep();
		}
	}

	void SoNS::Recruit(string _id) {
		for (auto& module : modules_) {
			module->Recruit(_id);
		}
	}

	void SoNS::Remove(string _id) {
		for (auto& module : modules_) {
			module->Remove(_id);
		}
	}

	void SoNS::RemoveWithUpdate(string _id) {
		for (auto& module : modules_) {
			module->RemoveWithUpdate(_id);
		}
	}

	SoNSStepResult SoNS::Step(
		double time,
		const vector<SoNSRobot>& perceivedNeighbors,
		const vector<struct SoNSMessage>& receivedMessages
	) {
		log_.str("");
		log_ << endl << "-------------------------------------------------------------------" << endl; // 记录信息
		log_ << "---I am " << myId_str_ << ", I belong to " << sonsId_str_ << ", My quality is " << sonsQuality_f_ << "------------------------------------" << endl; // 记录信息

		log_ << "--- raw input --------" << endl;
		//- debug print raw messages ---------------------------------------------
		for (const auto& message : receivedMessages) {
			log_ << "\tMessage : " << message.id << ", " << messager_.printHex(message.binary) << endl;
			log_ << messager_.printCmd(message.binary, "\t\t");
		}
		log_ << "\tneighbours : ----------------------" << endl;
		for (auto& pair : neighbors_mapRobot_) {
			log_ << "\t\t" << pair.first << endl;
		}

		//--------------------------------------------------------------

		messager_.clearMessages();
		messager_.OrganizeReceivedCommands(receivedMessages);
		UpdateNeighbors(perceivedNeighbors, time);

		sonsConnector.Step(time);
		/*
		for (auto& module : modules_) {
			module->Step(time, log);
		}
		*/

		log_ << "--- step result --------" << endl;
		log_ << "\tparent : --------" << endl;
		if (parent_RobotP_ != nullptr) log_ << "\t\t" << parent_RobotP_->id << " " << parent_RobotP_->heartbeatCD << endl;
		log_ << "\tchildren : ----------------------" << endl;
		for (auto& pair : children_mapRobotP_) {
			log_ << "\t\t" << pair.first << " " << pair.second->heartbeatCD << endl;
		}
		log_ << "\twaiting list: ----------------------" << endl;
		for (auto& pair : sonsConnector.m_WaitingList) {
			log_ << "\t\t" << pair.first << "\t " << pair.second.waitingTimeCountDown << endl;
		}

		SoNSStepResult result;
		map<string, vector<uint8_t>> messageMap = messager_.combineCommands();
		for (auto& pair : messageMap) result.messages.push_back({pair.first, pair.second});
		result.log = log_.str(); // 将ostringstream转换为string

		for (const auto& message : result.messages) {
			log_ << "\tSending : " << message.id << ", " << messager_.printHex(message.binary) << endl;
		}

		/*
		for (const auto& neighbour : neighbors_mapRobot_) {
			result.drawArrows.emplace_back(SoNSArrow::Color::BLUE, neighbour.second.GetPosition());
		}
		*/
		for (const auto& neighbour : children_mapRobotP_) {
			result.drawArrows.emplace_back(SoNSArrow::Color::RED, neighbour.second->GetPosition());
		}
		return result;
	}

	//--------------------------------------------------------------------------------------------------
	void SoNS::UpdateNeighbors(const vector<SoNSRobot>& perceivedNeighbors, double time) {
		for (const SoNSRobot& robot : perceivedNeighbors) {
			if (!ExistsInNeighbors(robot.id)) {
				// If it doesn't exist, add the robot
				neighbors_mapRobot_[robot.id] = robot;
				neighbors_mapRobot_[robot.id].transform = robot.transform;
				neighbors_mapRobot_[robot.id].seenCD = parameters_.seenCDTime;
			}
			else {
				// else update robot
				neighbors_mapRobot_[robot.id].transform = robot.transform;
				neighbors_mapRobot_[robot.id].seenCD = parameters_.seenCDTime;
			}
		}

		// remove outdated robots
		for (auto it = neighbors_mapRobot_.begin(); it != neighbors_mapRobot_.end();) {
			it->second.seenCD -= time; // Decrease the heartbeat by the elapsed time
			if (it->second.seenCD < 0) {
				if (ExistsInChildren(it->first) || ExistsInParent(it->first))
					messager_.sendCommand(it->first, CMessager::CommandType::BREAK, {});
				RemoveWithUpdate(it->first);
				it = neighbors_mapRobot_.erase(it);
			}
			else {
				it++;
			}
		}
	}

} // end of namespace