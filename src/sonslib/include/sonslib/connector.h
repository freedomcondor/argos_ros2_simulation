#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <sstream>

#include "sonsDataStruct.h"

namespace SoNSLib {
	class SoNSConnector {
	public:
		struct WaitingSoNSRobot {
			double waitingTimeCountDown;
			SoNSRobot* pRobot;
		};

		void Step(SoNSData& sonsData, double time, std::ostringstream& log);

	private:
		map<string, WaitingSoNSRobot> m_WaitingList;
		void UpdateWaitingList(double time);
		//void Remove(SoNSData& sonsData, string id);
		void Recruit(SoNSData& sonsData, string id);

		//------------------------------------------------------
		void pushRecruitMessage(vector<uint8_t>& _binary);
	};
}

#endif