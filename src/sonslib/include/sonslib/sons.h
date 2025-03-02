#ifndef SONS_H
#define SONS_H

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

#include "sonsDataStruct.h"
#include "messager.h"

namespace SoNSLib {
	class SoNS {
		public:
			SoNS();
			void SetId(string _myId, string _myType);
			struct SoNSData m_Data;
			struct SoNSStepResult Step(double time, const vector<SoNSRobot>& perceivedNeighbors, const vector<struct SoNSMessage>& receivedMessages);
		private:
			SoNSMessager m_Messager;
			string myId;
			string myType;
	};
}

#endif // SONS_H