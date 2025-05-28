#include "scaleManager.h"
#include "messager.h"
#include "sons.h"

namespace SoNSLib {

	void SoNSScaleManager::Init() {
		
	}

	void SoNSScaleManager::Step(double time) {
		// receive messages from children
		for (const auto& command : sons_->GetReceivedCommands()[CMessager::CommandType::SCALE]) {
			string fromId = command.id;
			if (sons_->ExistsInChildren(fromId)) {
				uint i = 0;
				parseScaleMessage(command.binary, i,
					sons_->children_mapRobotP_[fromId]->scale,
					sons_->children_mapRobotP_[fromId]->depth
				);

				if (sons_->children_mapRobotP_[fromId]->depth > sons_->parameters_.maxDepth) {
					sons_->log_ << "ScaleManager: " << fromId << " is too deep" << std::endl;
					if (sons_->parent_RobotP_ != nullptr) {
						sons_->RemoveWithUpdate(sons_->parent_RobotP_->id);
					}
				}
			}
		}

		// clear scale and depth for sum
		sons_->scale_.clear();
		sons_->depth_ = 0;

		// sum children scale to my scale and depth
		for (const auto& [id, robotp] : sons_->children_mapRobotP_) {
			// depth
			if (robotp->depth > sons_->depth_) {
				sons_->depth_ = robotp->depth;
			}

			// scale
			for (const auto& [key, value] : robotp->scale) {
				// check key2 doesn't exist in my scale
				if (sons_->scale_.find(key) == sons_->scale_.end()) {
					sons_->scale_[key] = 0;
				}
				// add key2
				sons_->scale_[key] += value;
			}
		}

		// add myself depth and scale
		sons_->depth_++;
		sons_->scale_[sons_->myType_str_]++;

		//send my scale to my parent
		if (sons_->parent_RobotP_ != nullptr) {
			sons_->messager_.sendCommand(sons_->parent_RobotP_->id, CMessager::CommandType::SCALE, generateScaleMessage(sons_->scale_, sons_->depth_));
		}

		// for debug: draw rings to show number of drones
		for (uint i = 0; i < sons_->scale_["drone"]; i++) {
			sons_->rings_.emplace_back(
				SoNSRing::Color::RED,
				CVector3(0, 0, 0.2 + i * 0.1),
				0.1
			);
		}
	}

	vector<uint8_t> SoNSScaleManager::generateScaleMessage(map<string, uint16_t> _scale, uint16_t _depth) {
		vector<uint8_t> content;
		CMessager::pushUint16(content, _depth);
		CMessager::pushUint16(content, _scale.size());
		for (auto& [key, value] : _scale) {
			CMessager::pushString(content, key);
			CMessager::pushUint16(content, value);
		}
		return content;
	}

	void SoNSScaleManager::parseScaleMessage(const vector<uint8_t>& _binary, uint& i, map<string, uint16_t>& _scale, uint16_t& _depth) {
		_depth = CMessager::parseUint16(_binary, i);
		_scale.clear();
		uint16_t size = CMessager::parseUint16(_binary, i);
		for (uint16_t j = 0; j < size; j++) {
			string key = CMessager::parseString(_binary, i);
			uint16_t value = CMessager::parseUint16(_binary, i);
			_scale[key] = value;
		}
	}
}
