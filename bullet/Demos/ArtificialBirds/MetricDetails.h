#pragma once
#ifndef METRIC_DETAILS_H__
#define METRIC_DETAILS_H__

#include "LinearMath/btAlignedObjectArray.h"

struct MetricDetails {
	int birdId;
	btAlignedObjectArray<btVector3> pelvisPosition;
	btAlignedObjectArray<btScalar> leftWingTorque;
	btAlignedObjectArray<btScalar> rightWingTorque;
	btAlignedObjectArray<btScalar> leftFeatherTorque;
	btAlignedObjectArray<btScalar> rightFeatherTorque;
};


#endif