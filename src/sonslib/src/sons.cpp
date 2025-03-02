#include "sons.h"
#include <iostream>
#include <random>
#include <sstream>
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
		std::ostringstream logMessage; // 使用ostringstream
		logMessage << "----------- I am " << myId << " Quality " << m_Data.sonsQuality << " ----------------------------" << endl; // 记录信息

		// 处理接收到的消息
		CVector3 vTotal = CVector3();
		int n = 0;

		for (const auto& message : receivedMessages) {
			int index = 0;
			double quality = m_Messager.parseDouble(message.binary, index); // 假设从索引0开始解析
			string itsId = m_Messager.parseString(message.binary, index);
			CVector3 velocity = m_Messager.parseCVector3(message.binary, index); // 假设从索引0开始解析
			logMessage << "    Parsed double from received message: " << message.id << ", id in message = " << itsId << " , " << quality << " velocity " << velocity << endl; // 记录信息

			vTotal += velocity;
			n++;
		}

		if (n != 0) vTotal = vTotal * (1.0/n);

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
		m_Messager.pushDouble(msg.binary, m_Data.sonsQuality);
		m_Messager.pushString(msg.binary, myId);
		m_Messager.pushCVector3(msg.binary, vTotal);

		struct SoNSStepResult res;
		res.outputVelocity = vTotal;
		res.messages.push_back(msg);
		res.logMessage = logMessage.str(); // 将ostringstream转换为string

		for (const SoNSRobot& robot : perceivedNeighbors) {
			res.drawArrows.emplace_back(SoNSArrow::Color::RED, robot.GetPosition());
		}
		res.drawArrows.emplace_back(SoNSArrow::Color::BLUE, vTotal);

		return res;
	}
}