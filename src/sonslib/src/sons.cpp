#include "sons.h"
#include <iostream>
#include <random>
using std::cout;
using std::endl;

namespace SoNSLib {
	SoNS::SoNS() {
		// 随机数生成器
		std::random_device rd; // 获取随机数种子
		std::mt19937 gen(rd()); // 使用梅森旋转算法生成随机数
		std::uniform_real_distribution<> dis(0.0, 1.0); // 定义范围为0到1的均匀分布

		m_Data.sonsQuality = dis(gen); // 生成随机数并赋值给sonsQuality
	}
	void SoNS::SetId(string _myId, string _myType) {
		myId = _myId;
		myType = _myType;
	}

	struct SoNSStepResult SoNS::Step(double time, const vector<SoNSRobot>& perceivedNeighbors, const vector<struct SoNSMessage>& receivedMessages) {
		cout << "----------- I am " << myId << " Quality " << m_Data.sonsQuality << " ----------------------------" << endl; // 打印解析出的double值
		// 处理接收到的消息
		for (const auto& message : receivedMessages) {
			double parsedValue = m_Messager.parseDouble(message.binary, 0); // 假设从索引0开始解析
			cout << "    Parsed double from received message: " << message.id << ", " << parsedValue << endl; // 打印解析出的double值
		}

		CVector3 vTotal = CVector3();
		double targetDistance = 3;

		for (const SoNSRobot& robot : perceivedNeighbors) {
			if ((robot.GetPosition().Length() != 0)) {
				CVector3 v = CVector3(robot.GetPosition()).Normalize() * (robot.GetPosition().Length() - targetDistance) * 0.06;
				if (v.Length() > 0.5) v = v.Normalize() * 0.5;
				vTotal += v;
			}
		}


		struct SoNSMessage msg;
		msg.id = SoNSBroadCastString;
		msg.binary = m_Messager.bindDouble(m_Data.sonsQuality);
		struct SoNSStepResult res;
		res.outputVelocity = vTotal;
		res.messages.push_back(msg);
		return res;
	}
}