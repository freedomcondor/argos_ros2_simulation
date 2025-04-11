#include "messager.h"
#include "sonsDataStruct.h"

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
}