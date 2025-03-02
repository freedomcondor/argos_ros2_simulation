#ifndef SONS_DATA_STRUCT_H
#define SONS_DATA_STRUCT_H

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

#include <vector>
using std::vector;

#include <string>
using std::string;

namespace SoNSLib {
	#define SoNSBroadCastString "BROADCAST"

	class SoNSRobot {
	public:
		string id;
		string type;
		CTransform transform;
		CVector3 GetPosition() const {return transform.m_Position;}
		CQuaternion GetOrientation() const {return transform.m_Orientation;}
		SoNSRobot(string _id, string _type, const CTransform& _transform):id(_id), type(_type), transform(_transform) {}
	};

	class SoNSData {
	public:
		string sonsId;
		double sonsQuality;

		std::vector<SoNSRobot> neighbors;
		SoNSRobot* parent;
		std::vector<SoNSRobot*> children;

		// TODO
		// depth
		// scale
	};

	struct SoNSMessage {
		string id;
		vector<uint8_t> binary;
	};

	struct SoNSStepResult {
		CVector3 outputVelocity;
		vector<struct SoNSMessage> messages;
	};

}

#endif