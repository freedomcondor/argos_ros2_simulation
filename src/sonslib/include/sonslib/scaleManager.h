#ifndef SCALEMANAGER_H
#define SCALEMANAGER_H

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

	class SoNSScaleManager : public SoNSModule {
	public:
		SoNSScaleManager(SoNS& sons) : SoNSModule(sons) {};
		void Init() override;
		void Step(double time);

	//private:
	public:
		//------------------------------------------------------
		vector<uint8_t> generateScaleMessage(map<string, uint16_t> _scale, uint16_t _depth);
		void parseScaleMessage(const vector<uint8_t>& _binary, uint& i, map<string, uint16_t>& _scale, uint16_t& _depth);

		map<string, uint16_t> AddScale(const map<string, uint16_t>& a, const map<string, uint16_t>& b);
		void AddScaleBtoA(map<string, uint16_t>& a, const map<string, uint16_t>& b);
		map<string, uint16_t> SubScale(const map<string, uint16_t>& a, const map<string, uint16_t>& b);
		void SubScaleBfromA(map<string, uint16_t>& a, const map<string, uint16_t>& b);
	};
}

#endif