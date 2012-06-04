#ifndef WINGBEAT_PATTERN_H__
#define WINGBEAT_PATTERN_H__

#include <string>
#include <vector>
#include "btBulletDynamicsCommon.h"
#include "../proto/wingbeat_pattern.pb.h"

using namespace std;

class WingbeatPattern {
	proto::WingbeatPattern pattern;

public:
	void Clear() {
		pattern.Clear();
	}

	void AddSample(float wingAngle, float featherAngle) {
		proto::JointAngleInfo* anglesElem = pattern.add_angles();
		anglesElem->set_wing(wingAngle);
		anglesElem->set_feather(featherAngle);
	}

	string DebugString() const {
		return pattern.DebugString();
	}
};

#endif