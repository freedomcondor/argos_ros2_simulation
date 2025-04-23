#include "flocking.h"
#include "sons.h"

#include "mathlib/math/vector3.h"

#include <algorithm>

using namespace swarmMathLib;

namespace SoNSLib {

	void SoNSFlocking::Step() {
		// 将邻居按距离排序
		std::vector<CVector3> sorted_neighbors;
		for (const auto& it : sons_->neighbors_mapRobot_) {
			CVector3 pos = it.second.GetPosition();
			double dis = pos.Length();
			if (dis > 0) { // 排除自身
				sorted_neighbors.emplace_back(pos);
			}
		}

		// sort sorted neighbors by distance
		std::sort(sorted_neighbors.begin(), sorted_neighbors.end(),
			[](const CVector3& a, const CVector3& b) -> bool {
				return a.Length() < b.Length();
			}
		);

		double threshold = 5;
		double speed_scalar = 0.1;
		int k = 5; // 设置k值，表示只考虑最近的k个邻居
		sons_->outputVelocity_ = CVector3(0, 0, 0);

		// 只对前k个邻居进行计算
		int count = 0;
		for (const auto& pos : sorted_neighbors) {
			if (count >= k) break; // 超过k个邻居则停止
			CVector3 normalized_pos = pos;
			normalized_pos.Normalize();
			double dis = pos.Length();
			if (dis > threshold) {
				sons_->outputVelocity_ += normalized_pos * speed_scalar * dis;
			} else {
				sons_->outputVelocity_ -= normalized_pos * speed_scalar * dis;
			}
			count++;
		}
	}

}