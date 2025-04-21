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
		RegisterModule(std::make_shared<SoNSConnector>(sonsConnector));
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

	void SoNS::Remove(string _id) {
		for (auto& module : modules_) {
			module->Remove(_id);
		}
	}

	SoNSStepResult SoNS::Step(
		double time,
		const vector<SoNSRobot>& perceivedNeighbors,
		const vector<struct SoNSMessage>& receivedMessages
	) {
		messager_.clearMessages();
		messager_.OrganizeReceivedCommands(receivedMessages);
		UpdateNeighbors(perceivedNeighbors, time);

		std::ostringstream log;
		sonsConnector.Step(time, log);
		/*
		for (auto& module : modules_) {
			module->Step(time, log);
		}
		*/
		SoNSStepResult result;
		for (const auto& neighbour : neighbors_mapRobot_) {
			result.drawArrows.emplace_back(SoNSArrow::Color::BLUE, neighbour.second.GetPosition());
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
				Remove(it->first);
				it = neighbors_mapRobot_.erase(it);
			}
			else {
				it++;
			}
		}
	}

} // end of namespace