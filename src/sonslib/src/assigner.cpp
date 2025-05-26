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

	void SoNSAssigner::Step(double time) {
		sons_->log_ << "first line of assigner " << endl;
		// check assign
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::ASSIGN]) {
			string fromId = command.id;
			if (sons_->ExistsInParent(fromId)) {
				sons_->assignTo_str_ = CMessager::parseString(command.binary);
				sons_->log_ << "parsing  assign command : " << sons_->assignTo_str_ << endl;
			}
		}
	}

}
