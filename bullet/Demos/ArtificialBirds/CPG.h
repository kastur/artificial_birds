#pragma once
#ifndef CPG_H__
#define CPG_H__

#include "LinearMath/btAlignedObjectArray.h"
#include <vector>
struct CPG {
	int birdId;
	btAlignedObjectArray<btScalar> reqWingFlappingAngle;
	btAlignedObjectArray<btScalar> reqFeatherAngleOfAttack1;
};

#endif