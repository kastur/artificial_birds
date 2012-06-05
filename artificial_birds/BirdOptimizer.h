#ifndef BIRD_OPTIMIZER_H__
#define BIRD_OPTIMIZER_H__

#include "btBulletDynamicsCommon.h"
#include "BigBirdLocalParams.h"
#include "../proto/proto.pb.h"
#include "BigBird.h"
#include "BigFeather.h"
#include "time.h"
#include <random>

typedef std::tr1::ranlux64_base_01 t_rand_eng;
typedef std::tr1::normal_distribution<float> t_rand_dist;

class BirdOptimizer {
public:
	BirdOptimizer(btDynamicsWorld* ownerWorld);
	~BirdOptimizer();
	void pretick(btScalar dt);
	void spawnBigBird();
	void removeBigBird();
	btVector3 getBirdPosition() { 
		if (m_bigbird)
			return m_bigbird->getPosition();
		return btVector3(0,3,0);
	}
protected:
	void perturbBestResult(const proto::BigBirdConstructionData& bestCPG, proto::BigBirdConstructionData* info);
	void evaluateCurrentGenerationBirds();

private:
	btDynamicsWorld* m_ownerWorld;
	BigBird* m_bigbird;
	btScalar m_time;  // keep track of time.
	btAlignedObjectArray<proto::TrajectoryData*> m_birdTrajectoryData;
	proto::TrajectoryData* m_currentTrajectoryData;
	btAlignedObjectArray<proto::BigBirdConstructionData*> m_birdInfos;
	proto::BigBirdConstructionData* m_currentBirdData;
	BigBirdLocalParams* m_currentBirdLocalParam;
	proto::BigBirdConstructionData* m_currentBestInfo;
	int m_numBirdsPerGeneration;
	int m_numGeneration;
	int m_giveBirdThisId;
	int m_numPoints;

	t_rand_eng m_rand_eng;
	t_rand_dist m_rand_dist;

	proto::BirdOptimizerData m_result_data;
	float m_perturb_scaler;
};

#endif