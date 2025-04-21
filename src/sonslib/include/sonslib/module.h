#ifndef MODULE_H
#define MODULE_H

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
		virtual void Remove(string _id) {}

	protected:
		SoNS* sons_;
	};
}

#endif

