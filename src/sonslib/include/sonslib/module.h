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
		virtual void Step(double time) { (void)time; }
		virtual void PostStep() {}
		virtual void Recruit(std::string _id) { (void)_id; }
		virtual void Assign(std::string _child_id, std::string _to_id) { (void)_child_id; (void)_to_id; }
		virtual void AddParent(std::string _id) { (void)_id; }
		virtual void AddChild(std::string _id) { (void)_id; }
		virtual void Remove(std::string _id) { (void)_id; }
		virtual void RemoveWithUpdate(std::string _id) { (void)_id; }

	protected:
		SoNS* sons_;
	};
}

#endif

