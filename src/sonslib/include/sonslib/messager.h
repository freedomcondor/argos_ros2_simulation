#ifndef MESSAGE_H
#define MESSAGE_H

#include <cstring> // for memcpy

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <map>
using std::map;
#include <sstream>

namespace SoNSLib {
	class CMessager {
	public:
		inline void clearMessages() {
			m_commandsToSend.clear();
			m_ReceivedCommands.clear();
		}

		//static const uint8_t MessageHeader = 0xCC;
		#define MessageHeader 0xCC

		enum CommandType {
			RECRUIT,
			ACKNOWLEDGEMENT,
			UPDATE,
			BREAK,
			HEARTBEAT,
		};

		struct Command {
			string id;
			CommandType type;
			vector<uint8_t> binary;
			Command(string _id, CommandType _type, vector<uint8_t> _binary):id(_id),type(_type),binary(_binary) {}
		};

		// cmd --------------------------------------------------------------
		void sendCommand(const string& _id, CommandType _type, const vector<uint8_t>& _content);
		map<string, vector<uint8_t>> combineCommands();
		void OrganizeReceivedCommands(const vector<struct SoNSMessage>& receivedMessages);
		static string printCommandType(CommandType _type);

		// message --------------------------------------------------------------
		static inline string printHex(const vector<uint8_t>& _content) {
			std::ostringstream hexlog;
			for (const uint8_t byte : _content) {
				// Convert byte to hex string with leading zero if needed
				hexlog << std::hex << std::uppercase;
				if (static_cast<int>(byte) < 16) hexlog << "0";
				hexlog << static_cast<int>(byte) << " ";
			}
			hexlog << std::dec;
			return hexlog.str();
		}

		string static printCmd(const vector<uint8_t>& _content, string indents);

		inline static void pushCommand(vector<uint8_t>& _binary, CommandType _type, const vector<uint8_t>& _content) {
			_binary.push_back(MessageHeader);
			pushCommandType(_binary, _type);
			pushUint16(_binary, _content.size());
			_binary.insert(_binary.end(), _content.begin(), _content.end());
		}
		inline static void parseCommand(const vector<uint8_t>& _binary, uint& i, CommandType& _type, vector<uint8_t>& _content) {
			// check messageheader
			if (_binary[i++] != MessageHeader) return;
			_type = parseCommandType(_binary, i);
			uint16_t contentSize = parseUint16(_binary, i);
			_content.insert(_content.end(), _binary.begin() + i, _binary.begin() + i + contentSize);
			i += contentSize;
		}
		// message type --------------------------------------------------------------
		inline static void pushCommandType(vector<uint8_t>& _binary, CommandType _type) {
			_binary.push_back(static_cast<uint8_t>(_type));
		}

		inline static CommandType parseCommandType(const vector<uint8_t>& _binary, uint& i) {
			CommandType type = static_cast<CommandType>(_binary[i++]);
			return type;
		}

		// uint16 --------------------------------------------------------------
		inline static void pushUint16(vector<uint8_t>& _binary, uint16_t _n) {
			uint8_t bytes[2];
			bytes[0] = static_cast<uint8_t>(_n & 0xFF);         // Lower byte
			bytes[1] = static_cast<uint8_t>((_n >> 8) & 0xFF);  // Higher byte
			_binary.insert(_binary.end(), bytes, bytes + 2);
		}

		inline static uint16_t parseUint16(const vector<uint8_t>& _binary, uint& i) {
			uint16_t result = 0;
			if (i + sizeof(uint16_t) <= _binary.size()) {
				result |= static_cast<uint16_t>(_binary[i++]); // Lower byte
				result |= static_cast<uint16_t>(_binary[i++]) << 8; // Higher byte
			}
			return result;
		}

		// double --------------------------------------------------------------
		inline static void pushDouble(vector<uint8_t>& _binary, double _d) {
			vector<uint8_t> res(reinterpret_cast<uint8_t*>(&_d), reinterpret_cast<uint8_t*>(&_d) + sizeof(double));
			_binary.insert(_binary.end(), res.begin(), res.end());
		}

		inline static double parseDouble(const vector<uint8_t>& _binary, uint& i) {
			double result;
			if (i + sizeof(double) <= _binary.size()) {
				std::memcpy(&result, _binary.data() + i, sizeof(double));
				i += sizeof(double);
				return result;
			} else {
				return 0;
			}
		}

		// CVector3 --------------------------------------------------------------
		inline static void pushCVector3(vector<uint8_t>& _binary, const CVector3& _v) {
			pushDouble(_binary, _v.GetX());
			pushDouble(_binary, _v.GetY());
			pushDouble(_binary, _v.GetZ());
		}

		inline static CVector3 parseCVector3(const vector<uint8_t>& _binary, uint& i) {
			double X = parseDouble(_binary, i);
			double Y = parseDouble(_binary, i);
			double Z = parseDouble(_binary, i);
			return CVector3(X, Y, Z);
		}

		// String --------------------------------------------------------------
		void static pushString(vector<uint8_t>& binary, const string& str) {
			size_t length = str.length();
			binary.push_back(static_cast<uint8_t>(length)); // 假设长度不超过255
			binary.insert(binary.end(), str.begin(), str.end());
		}

		string static parseString(const vector<uint8_t>& binary, uint& index) {
			uint8_t length = static_cast<uint8_t>(binary[index++]); // 假设长度不超过255
			string result(binary.begin() + index, binary.begin() + index + length);
			index += length; // 更新索引
			return result;
		}

		// member --------------------------------------------------------------
		map<CommandType, vector<Command>>& GetReceivedCommands() {return m_ReceivedCommands;}
	private :
		vector<Command> m_commandsToSend;
		map<CommandType, vector<Command>> m_ReceivedCommands;
	};
}

#endif