#ifndef BIRD_H__
#define BIRD_H__

#include "../proto/proto.pb.h"
#include "BigBirdLocalParams.h"
#include "BigBird.h"

struct Bird {
	BigBird* m_bigbird;
	BigBirdLocalParams* m_birdLocalParams;
	proto::BigBirdConstructionData* m_birdConstData;
};


#endif