#include "connector.h"
#include "messager.h"

#include <iostream>
using std::endl;

namespace SoNSLib {
	void SoNSConnector::Step(SoNSData& sonsData, double time, std::ostringstream& log) {
		UpdateWaitingList(time);

		// recruit all
		for (auto& pair : sonsData.neighbors) {
			if (m_WaitingList.find(pair.first) == m_WaitingList.end()) {
				// Recruit the robot
				// Assuming there is a method to recruit in the commented out section
				Recruit(sonsData, pair.first);
			}
		}

		log << "waiting list: ------------" << endl;
		for (const auto& pair : m_WaitingList) {
			log << "    Robot ID: " << pair.first << ", Waiting Time Countdown: " << pair.second.waitingTimeCountDown << endl;
		}
	};

	void SoNSConnector::UpdateWaitingList(double time) {
		for (auto& pair : m_WaitingList) {
			WaitingSoNSRobot& waitingRobot = pair.second;
			waitingRobot.waitingTimeCountDown -= time;
			if (waitingRobot.waitingTimeCountDown < 0)
				m_WaitingList.erase(pair.first);
		}
	}

	void SoNSConnector::Recruit(SoNSData& sonsData, string id) {
		WaitingSoNSRobot waitingSoNSRobot;
		waitingSoNSRobot.pRobot = &(sonsData.neighbors[id]);
		waitingSoNSRobot.waitingTimeCountDown = sonsData.parameters.recruitWaitingTime;
		m_WaitingList[id] = waitingSoNSRobot;

		struct SoNSMessage message; message.id = id;
		sonsData.messagesToSend[id] = message;
		pushRecruitMessage(sonsData.messagesToSend[id].binary);
	}

	//------------------------------------------------------
	void SoNSConnector::pushRecruitMessage(vector<uint8_t>& _binary) {
		SoNSMessager::pushString(_binary, "recruit");
	}
}
