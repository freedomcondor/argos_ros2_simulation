#ifndef ASSIGNER_H
#define ASSIGNER_H

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

	class SoNSAssigner: public SoNSModule {
	public:
		SoNSAssigner(SoNS& sons) : SoNSModule(sons) {};
		void Step(double time);
		void AddParent(string id) override;
		void AddChild(string id) override;
		void Remove(string id) override;
		void Assign(std::string _child_id, std::string _to_id) override;
	};
}

#endif