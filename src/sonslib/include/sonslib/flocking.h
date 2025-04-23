#ifndef FLOCKING_H
#define FLOCKING_H

#include "module.h"

namespace SoNSLib {

	class SoNSFlocking : public SoNSModule {
	public:
		SoNSFlocking(SoNS& sons) : SoNSModule(sons) {}
		void Step() override;
	}; // endof class Flocking

} // endof namespace SoNSLib

#endif