#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <sstream>

#include "sonsDataStruct.h"

namespace SoNSLib {
	class SoNSConnector {
	public:
		void Step(SoNSData& sonsData, double time, std::ostringstream& log);
	};
}

#endif