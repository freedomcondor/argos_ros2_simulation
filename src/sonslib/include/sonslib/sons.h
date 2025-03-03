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
#include "connector.h"

namespace SoNSLib {
	class SoNS {
		public:
			SoNS();
			void SetId(string _myId, string _myType);
			struct SoNSStepResult Step(double time, const vector<SoNSRobot>& perceivedNeighbors, const vector<struct SoNSMessage>& receivedMessages);
		private:
			struct SoNSData m_Data;

			void UpdateNeighbors(const vector<SoNSRobot>& perceivedNeighbors, double time);

			SoNSConnector connector;
	};
}

#endif // SONS_H