#include "BirdOptimizer.h"

BirdOptimizer::BirdOptimizer(btDynamicsWorld* ownersWorld, int numBirds) 
: m_ownerWorld(ownersWorld), m_numBirds(numBirds) {
	
	m_time = 0;
	m_bigbird = 0;
	m_currentBirdMetricDetails = 0;
	spawnBigBird(btVector3(0,0,0));
}

BirdOptimizer::~BirdOptimizer() {
	removeBigBird();
	for (int ii = 0 ; ii < m_birdMetricDetails.size(); ii++) {
		delete m_birdMetricDetails.at(ii);
	}
}

void BirdOptimizer::spawnBigBird(const btVector3& startOffset)
{
	BigBirdConstructionInfo info;
	
	info.birdId = 0;

	info.startTransform.setIdentity();
	info.startTransform.setOrigin(startOffset);

	info.hoistTransform.setIdentity();
	info.hoistTransform.setOrigin(startOffset);
	info.hoistAngleXY = 45.f;
	info.hoistAngleZXY = 90.f;

	info.pelvisHalfLength = 0.5f;
	info.wingHalfLength = 0.5f;
	info.hoistMass = 0.0f;
	info.pelvisMass = 2.0f;
	info.wingMass = 0.5f;
	info.pelvisRelPosToAttachWing = btVector3(0.f, 0.f, 0.f);
	info.featherRelPosToAttachFeather = btVector3(0.f, 0.f, 0.f);
	info.wingFlapHingeLimit = 70.f;
	info.featherAoAHingeLimit = 40.f;
	info.featherAoAMotorMaxImpulse = 1000.0f;
	info.wingFlapMotorMaxImpulse = 1000.0f;
	info.wingFlapFrequency = 3.5f;

	info.numPoints = 180;
	info.randSeed = (unsigned int)time(NULL);
	srand(info.randSeed);

	fillWithRandomNumbers(&(info.birdCPG.reqWingFlappingAngle),-info.wingFlapHingeLimit,info.wingFlapHingeLimit,info.numPoints);
	fillWithRandomNumbers(&(info.birdCPG.reqFeatherAngleOfAttack1),-info.featherAoAHingeLimit,info.featherAoAHingeLimit,info.numPoints);
	info.birdCPG.birdId = info.birdId;

	m_currentBirdMetricDetails = new MetricDetails();
	m_currentBirdMetricDetails->birdId = info.birdId;
	m_bigbird = new BigBird(m_ownerWorld, info);
}

//void BirdOptimizer::fillWithRandomNumbers(std::vector<btScalar> arrScalar, btScalar minValue, btScalar maxValue, int numPoints)
void BirdOptimizer::fillWithRandomNumbers(btAlignedObjectArray<btScalar>* arrScalar, btScalar minValue, btScalar maxValue, int numPoints)
{
	for (int ii = 0 ; ii < numPoints; ++ii) {
		arrScalar->push_back(minValue + (((double)rand())/RAND_MAX)*(maxValue-minValue));
	}
}

void BirdOptimizer::pretick(btScalar dt) {
	m_time += dt;
	
	if (m_bigbird) {
		m_currentBirdMetricDetails->pelvisPosition.push_back(m_bigbird->getPosition());
		m_bigbird->pretick(dt);
	}
}

void BirdOptimizer::removeBigBird() {
	if (m_bigbird) {
		delete m_bigbird;
		m_bigbird = 0;
		m_birdMetricDetails.push_back(m_currentBirdMetricDetails);
		m_currentBirdMetricDetails = 0;
	}
}