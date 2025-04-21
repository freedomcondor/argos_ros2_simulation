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
		void PreStep() override;
		void Step(double time);
		void UpdateSoNSID();

		void Recruit(string id) override;
		void Remove(string id) override;
		void RemoveWithUpdate(string id) override;

	//private:
	public:
		struct WaitingSoNSRobot {
			double waitingTimeCountDown;
			SoNSRobot* pRobot;
		};
		map<string, WaitingSoNSRobot> m_WaitingList;
		void UpdateWaitingList(double time);

		map<string, double> branchQualities_;
		//------------------------------------------------------
		vector<uint8_t> generateRecruitMessage(string sons_id, double sons_quality);
		void parseRecruitMessage(const vector<uint8_t>& _binary, uint& i, string& _his_id, double& _his_quality);
		vector<uint8_t> generateBranchQualities(const map<string, double>& _branchQualities);
		void parseBranchQualities(const vector<uint8_t>& _binary, uint& i, map<string, double>& _branchQualities);
	};
}

#endif