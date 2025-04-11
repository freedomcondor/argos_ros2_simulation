#ifndef SONS_DATA_STRUCT_H
#define SONS_DATA_STRUCT_H

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

#include "messager.h"

#include <vector>
using std::vector;

#include <map>
using std::map;

#include <string>
using std::string;

namespace SoNSLib {
	#define SoNSBroadCastString "BROADCAST"

	//- structs ---------------------------------------------------------------------------------------------
	struct SoNSParameters {
		double heartbeatTime = 3;
		double recruitWaitingTime = 1;
	};

	struct SoNSRobot {
	public:
		string id;
		string type;
		double heartbeat;
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

	//- SoNS data -------------------------------------------------------------------------------------------
	struct SoNSData {
	public:
		string myId;
		string myType;

		string sonsId;
		double sonsQuality;

		map<string, SoNSRobot> neighbors;
		SoNSRobot* parent;
		map<string, SoNSRobot*> children;

		SoNSParameters parameters;

		CMessager sonsMessager;

		// TODO
		// depth
		// scale
		map<CMessager::CommandType, vector<CMessager::Command>>& GetReceivedCommands() {return sonsMessager.GetReceivedCommands();}
	};

	struct SoNSStepResult {
		CVector3 outputVelocity;
		vector<SoNSMessage> messages;
		vector<SoNSArrow> drawArrows;
		string log;
	};

}

#endif