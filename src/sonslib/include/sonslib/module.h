#ifndef MODULE_H
#define MODULE_H

#include<string>

namespace SoNSLib {
	class SoNS;

	class SoNSModule {
	public:
		SoNSModule(SoNS& sons) : sons_(&sons) {}
		virtual ~SoNSModule() = default;
		virtual void Init() {}
		virtual void PreStep() {}
		virtual void Step() {}
		virtual void PostStep() {}
		virtual void Recruit(std::string _id) { (void)_id; }
		virtual void Remove(std::string _id) { (void)_id; }
		virtual void RemoveWithUpdate(std::string _id) { (void)_id; }

	protected:
		SoNS* sons_;
	};
}

#endif

