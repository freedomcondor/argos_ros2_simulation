#ifndef SONS_H
#define SONS_H

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <memory> // for shared ptr

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

#include "module.h"
#include "messager.h"
#include "connector.h"

namespace SoNSLib {
	//- structs ---------------------------------------------------------------------------------------------
	struct SoNSParameters {
		double seenCDTime = 3;
		double heartbeatCDTime = 3;
		double recruitWaitingTime = 1;
	};

	struct SoNSRobot {
	public:
		string id;
		string type;
		double heartbeatCD;
		double seenCD;
		CTransform transform;
		CVector3 GetPosition() const {return transform.m_Position;}
		CQuaternion GetOrientation() const {return transform.m_Orientation;}
		SoNSRobot() {}
		SoNSRobot(string _id, string _type, const CTransform& _transform):
			id(_id), type(_type), transform(_transform) {}
	};

	struct SoNSMessage {
		string id;
		vector<uint8_t> binary;
	};

	struct SoNSArrow{
		enum Color { RED, GREEN, BLUE, YELLOW, BLACK, WHITE };
		Color color;
		CVector3 arrow;
		SoNSArrow(Color _color, const CVector3& _arrow):color(_color),arrow(_arrow) {}
	};

	struct SoNSStepResult {
		CVector3 outputVelocity;
		vector<SoNSMessage> messages;
		vector<SoNSArrow> drawArrows;
		string log;
	};

	//- classes ---------------------------------------------------------------------------------------------
	class SoNSModule;

	class SoNS {
		public:
			SoNS();
			void RegisterModule(std::shared_ptr<SoNSModule> module);

			void Init(string _myId, string _myType);
			void PreStep();
			void PostStep();
			SoNSStepResult Step(
				double time,
				const vector<SoNSRobot>& perceivedNeighbors,
				const vector<struct SoNSMessage>& receivedMessages
			);
			void Recruit(string _id);
			void Remove(string _id);
			void RemoveWithUpdate(string _id);

		private:
			void UpdateNeighbors(const vector<SoNSRobot>& perceivedNeighbors, double time);

		// sons data
		public:
			// ids
			string myId_str_;
			string myType_str_;
			string sonsId_str_;
			double sonsQuality_f_;

			// neighbours
			map<string, SoNSRobot> neighbors_mapRobot_;
			SoNSRobot* parent_RobotP_;
			map<string, SoNSRobot*> children_mapRobotP_;
			bool ExistsInNeighbors(string _id) {return neighbors_mapRobot_.find(_id) != neighbors_mapRobot_.end();}
			bool ExistsInChildren(string _id) {return children_mapRobotP_.find(_id) != children_mapRobotP_.end();}
			bool ExistsInParent(string _id) {return parent_RobotP_ != nullptr && parent_RobotP_->id == _id;}

			// utils
			SoNSParameters parameters_;
			std::ostringstream log_;
			CMessager messager_;
			map<CMessager::CommandType, vector<CMessager::Command>>& GetReceivedCommands() {return messager_.GetReceivedCommands();}

			// modules
			SoNSConnector sonsConnector;

		private:
			std::vector<std::shared_ptr<SoNSModule>> modules_;
	};
} // end of namespace SoNSLib

#endif // SONS_H