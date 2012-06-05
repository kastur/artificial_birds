#ifndef BIG_BIRD_CONSTRUCTION_INFO__
#define BIG_BIRD_CONSTRUCTION_INFO__

#include "../proto/proto.pb.h"
#include "btBulletDynamicsCommon.h"

struct BigBirdLocalParams {

	int birdId;
	
	//Hoist offset
	btTransform hoistTransform;

	// Start Offset
	btTransform startTransform;
	
};

#endif