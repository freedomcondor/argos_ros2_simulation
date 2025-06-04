#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <sstream>
#include <map>
using std::map;
#include <vector>
using std::vector;
#include <string>
using std::string;

#include "module.h"

namespace SoNSLib {
	class SoNS;
    struct SoNSRobot;

	class SoNSConnector : public SoNSModule {
	public:
		SoNSConnector(SoNS& sons) : SoNSModule(sons) {};
		void Init() override;
		void UpdateSoNSID();
		void Step(double time);

		void Recruit(string id) override;
		void AddParent(string id) override;
		void AddChild(string id) override;
		void Remove(string id) override;
		void RemoveWithUpdate(string id) override;

	//private:
	public:
		struct WaitingSoNSRobot {
			double waitingTimeCountDown;
			SoNSRobot* pRobot;
		};
		map<string, double> m_lockmap;
		string sonsLastId_str_;
		double sonsLastQuality_f_;
		map<string, WaitingSoNSRobot> m_WaitingList;
		void UpdateWaitingList(double time);
		//------------------------------------------------------
		vector<uint8_t> generateRecruitMessage(string sons_id, double sons_quality);
		void parseRecruitMessage(const vector<uint8_t>& _binary, uint& i, string& _his_id, double& _his_quality);
	};
}

#endif