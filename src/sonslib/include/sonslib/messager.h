#ifndef MESSAGE_H
#define MESSAGE_H

#include "sonsDataStruct.h"

#include <cstring>

namespace SoNSLib {
	class SoNSMessager {
	public:
		inline vector<uint8_t> bindDouble(double _d) {
			vector<uint8_t> res(reinterpret_cast<uint8_t*>(&_d), reinterpret_cast<uint8_t*>(&_d) + sizeof(double));
			return res;
		}

		inline void pushBindDouble(vector<uint8_t>& _binary, double _d) {
			vector<uint8_t> res(reinterpret_cast<uint8_t*>(&_d), reinterpret_cast<uint8_t*>(&_d) + sizeof(double));
			_binary.insert(_binary.end(), res.begin(), res.end());
		}

		inline double parseDouble(const vector<uint8_t>& _binary, int i) const {
			double result;
			if (i + sizeof(double) <= _binary.size()) {
				std::memcpy(&result, _binary.data() + i, sizeof(double));
				return result;
			} else {
				return 0;
			}
		}
	};
}

#endif