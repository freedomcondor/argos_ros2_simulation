#include "connector.h"

#include <iostream>
using std::endl;

namespace SoNSLib {
	void SoNSConnector::Step(SoNSData& sonsData, double time, std::ostringstream& log) {
		log << "I am connector" << endl;
	};
}
