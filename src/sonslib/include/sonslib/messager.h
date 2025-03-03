#ifndef MESSAGE_H
#define MESSAGE_H

#include <cstring> // for memcpy

#include "sonsDataStruct.h"

#include "mathlib/math/vector3.h"
#include "mathlib/math/transform.h"
using namespace swarmMathLib;


namespace SoNSLib {
	class SoNSMessager {
	public:
		// double --------------------------------------------------------------
		inline static void pushDouble(vector<uint8_t>& _binary, double _d) {
			vector<uint8_t> res(reinterpret_cast<uint8_t*>(&_d), reinterpret_cast<uint8_t*>(&_d) + sizeof(double));
			_binary.insert(_binary.end(), res.begin(), res.end());
		}

		inline static double parseDouble(const vector<uint8_t>& _binary, int& i) {
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

		inline static CVector3 parseCVector3(const vector<uint8_t>& _binary, int& i) {
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

		string static parseString(const vector<uint8_t>& binary, int& index) {
			size_t length = static_cast<size_t>(binary[index++]); // 假设长度不超过255
			string result(binary.begin() + index, binary.begin() + index + length);
			index += length; // 更新索引
			return result;
		}
	};
}

#endif