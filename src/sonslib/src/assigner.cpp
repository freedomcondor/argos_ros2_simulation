#include "assigner.h"

#include "messager.h"
#include "sons.h"

#include <iostream>
using std::endl;
#include <string>
using std::string;

namespace SoNSLib {

	void SoNSAssigner::Assign(string _child_id, string _to_id) {
		if (sons_->ExistsInChildren(_child_id)) {
			vector<uint8_t> content;
			sons_->messager_.pushString(content, _to_id);
			sons_->messager_.sendCommand(_child_id, CMessager::CommandType::ASSIGN, content);
		}
	}

	void SoNSAssigner::AddParent(string _id) {
		sons_->assignTo_str_ = "";
	}

	void SoNSAssigner::AddChild(string _id) {
		if (sons_->assignTo_str_ == _id) {
			sons_->assignTo_str_ = "";
		}
	}

	void SoNSAssigner::Remove(string _id) {
		if (sons_->ExistsInParent(_id)) {
			sons_->assignTo_str_ = "";
		}
	}

	void SoNSAssigner::Step(double time) {
		// check assign
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::ASSIGN]) {
			string fromId = command.id;
			if (sons_->ExistsInParent(fromId)) {
				sons_->assignTo_str_ = CMessager::parseString(command.binary);
			}
		}
	}

}
