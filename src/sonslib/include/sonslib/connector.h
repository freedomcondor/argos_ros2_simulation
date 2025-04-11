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

		void Initialize(SoNSData& sonsData);
		void Step(SoNSData& sonsData, double time, std::ostringstream& log);

	private:
		map<string, WaitingSoNSRobot> m_WaitingList;
		void UpdateWaitingList(double time);
		//void Remove(SoNSData& sonsData, string id);
		void Recruit(SoNSData& sonsData, string id);

		//------------------------------------------------------
		vector<uint8_t> generateRecruitMessage(string sons_id, double sons_quality);
		void parseRecruitMessage(const vector<uint8_t>& _binary, uint& i, string& _his_id, double& _his_quality);
	};
}

#endif