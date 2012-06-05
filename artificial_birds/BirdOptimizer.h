#ifndef BIRD_OPTIMIZER_H__
#define BIRD_OPTIMIZER_H__

#include "btBulletDynamicsCommon.h"
#include "BigBirdConstructionInfo.h"
#include "BigBird.h"
#include "BigFeather.h"
#include "MetricDetails.h"
#include "time.h"

class BirdOptimizer {
public:
	BirdOptimizer(btDynamicsWorld* ownerWorld, int numBirds);
	~BirdOptimizer();
	void pretick(btScalar dt);
	void spawnBigBird(const btVector3& startOffset);
	void removeBigBird();
	btVector3 getBirdPosition() { 
		if (m_bigbird)
			return m_bigbird->getPosition();
		return btVector3(0,0,0);
	}
protected:
	void fillWithRandomNumbers(btAlignedObjectArray<btScalar>* arrScalar, btScalar minValue, btScalar maxValue, int numPoints);
	//void spawnBigBird(const btVector3& startOffset);
	//void removeBird();
	void perturbBestResult(btAlignedObjectArray<btScalar>* arrBase, btAlignedObjectArray<btScalar>* arrChanged, btScalar minValue, btScalar maxValue, int numPoints);
	void evaluateCurrentGenerationBirds();

private:
	btDynamicsWorld* m_ownerWorld;
	BigBird* m_bigbird;
	btScalar m_time;  // keep track of time.
	btAlignedObjectArray<struct MetricDetails*> m_birdMetricDetails;
	MetricDetails* m_currentBirdMetricDetails;
	btAlignedObjectArray<struct CPG*> m_birdCPGs;
	CPG* m_currentBestCPG;
	int m_numBirdsPerGeneration;
	int m_numGeneration;
	int m_giveBirdThisId;
};

#endif