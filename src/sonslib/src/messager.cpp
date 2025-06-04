#include "messager.h"
#include "sons.h"

namespace SoNSLib {
	// cmd --------------------------------------------------------------
	void CMessager::sendCommand(const string& _id, CommandType _type, const vector<uint8_t>& _content) {
		m_commandsToSend.emplace_back(_id, _type, _content);
	}    

	map<string, vector<uint8_t>> CMessager::combineCommands() {
		map<string, vector<uint8_t>> messagesIndex;

		for (const Command& cmd : m_commandsToSend) {
			pushCommand(messagesIndex[cmd.id], cmd.type, cmd.binary);
		}

		return messagesIndex;
	}

	void CMessager::OrganizeReceivedCommands(const vector<struct SoNSMessage>& receivedMessages) {
		m_ReceivedCommands.clear();
		for (const struct SoNSMessage& message : receivedMessages) {
			string fromID = message.id;

			const vector<uint8_t>& binary = message.binary;
			uint i = 0;
			while (i < binary.size()) {
				CommandType type;
				vector<uint8_t> content;
				parseCommand(binary, i, type, content);
				m_ReceivedCommands[type].emplace_back(fromID, type, content);
			}
		}
	}

	string CMessager::printCommandType(CommandType _type) {
		switch (_type) {
			case CommandType::RECRUIT:
				return "RECRUIT";
			case CommandType::ACKNOWLEDGEMENT:
				return "ACKNOWLEDGEMENT"; 
			case CommandType::UPDATE:
				return "UPDATE";
			case CommandType::BREAK:
				return "BREAK";
			case CommandType::HEARTBEAT:
				return "HEARTBEAT";
			case CommandType::SCALE:
				return "SCALE";
			case CommandType::ASSIGN:
				return "ASSIGN";
			default:
				return "UNKNOWN";
		}
	}

	string CMessager::printCmd(const vector<uint8_t>& _content, string indents) {
		std::ostringstream cmdlog;
		uint i = 0;
		while (i < _content.size()) {
			CommandType type;
			vector<uint8_t> cmd;
			parseCommand(_content, i, type, cmd);
			cmdlog << indents << printCommandType(type) << ", ";

			uint contentI = 0;
			uint16_t qualitiesSize;
			switch (type) {
				case CommandType::RECRUIT:
					cmdlog << parseString(cmd, contentI) << " "
					       << parseDouble(cmd, contentI);
					break;
				case CommandType::UPDATE:
					cmdlog << parseString(cmd, contentI) << " "
					       << parseDouble(cmd, contentI);
					break;
				case CommandType::ACKNOWLEDGEMENT: break;
				case CommandType::BREAK:           break;
				case CommandType::HEARTBEAT:
					qualitiesSize = parseUint16(cmd, contentI);
					for (uint16_t j = 0; j < qualitiesSize; ++j) {
						cmdlog << parseString(cmd, contentI) << " "
						       << parseDouble(cmd, contentI) << ", ";
					}
					break;
				case CommandType::SCALE:
					{
						cmdlog << "<depth> = " << parseUint16(cmd, contentI) << " ";
						uint16_t size = parseUint16(cmd, contentI);
						cmdlog << "<scale> = " << size << " ";
						for (int i = 0; i < size; i++) {
							cmdlog << parseString(cmd, contentI) << " "
								<< parseUint16(cmd, contentI);
						}
					}
					break;
				case CommandType::ASSIGN:
					cmdlog << parseString(cmd, contentI);
					break;
				default:  break;
			}

			cmdlog << std::endl;
		}

		return cmdlog.str();
	}

}
