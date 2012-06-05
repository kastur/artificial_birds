#include "BirdOptimizer.h"

BirdOptimizer::BirdOptimizer(btDynamicsWorld* ownersWorld, int numBirds) 
: m_ownerWorld(ownersWorld), m_numBirdsPerGeneration(numBirds) {
	
	m_time = 0;
	m_bigbird = 0;
	m_currentBirdMetricDetails = 0;
	m_numGeneration = -1;
	m_currentBestCPG = 0;
	m_giveBirdThisId = 0;
	spawnBigBird(btVector3(0,0,0));
}

BirdOptimizer::~BirdOptimizer() {
	removeBigBird();
	for (int ii = 0 ; ii < m_birdMetricDetails.size(); ii++) {
		delete m_birdMetricDetails.at(ii);
	}

	for (int ii = 0; ii < m_birdCPGs.size(); ii++) {
		if (m_birdCPGs.at(ii) == m_currentBestCPG) continue;
		delete m_birdCPGs.at(ii);
	}
	if (m_currentBestCPG)
		delete m_currentBestCPG;
}

void BirdOptimizer::spawnBigBird(const btVector3& startOffset)
{
	if ((m_giveBirdThisId % m_numBirdsPerGeneration) == 0) {
		m_numGeneration++;
		if (m_numGeneration > 0) {
			evaluateCurrentGenerationBirds();
		}
	}

	BigBirdConstructionInfo info;
	
	info.birdId = m_giveBirdThisId++;

	info.startTransform.setIdentity();
	info.startTransform.setOrigin(startOffset);

	info.hoistTransform.setIdentity();
	info.hoistTransform.setOrigin(startOffset);
	info.hoistAngleXY = 30.f;
	info.hoistAngleZXY = 90.f;

	info.pelvisHalfLength = 0.5f;
	info.wingHalfLength = 0.5f;
	info.hoistMass = 0.0f;
	info.pelvisMass = 2.0f;
	info.wingMass = 0.5f;
	info.pelvisRelPosToAttachWing = btVector3(0.f, 0.f, 0.f);
	info.featherRelPosToAttachFeather = btVector3(0.f, 0.f, 0.f);
	info.wingFlapHingeLimit = 90.f;
	info.featherAoAHingeLimit = 90.f;
	info.featherAoAMotorMaxImpulse = 1000.0f;
	info.wingFlapMotorMaxImpulse = 1000.0f;
	info.wingFlapFrequency = 3.5f;

	info.numPoints = 180;
	info.randSeed = (unsigned int)time(NULL);
	srand(info.randSeed);

	info.birdCPG = new CPG();

	if(m_numGeneration == 0) {
		fillWithRandomNumbers(&(info.birdCPG->reqWingFlappingAngle),-info.wingFlapHingeLimit,info.wingFlapHingeLimit,info.numPoints);
		fillWithRandomNumbers(&(info.birdCPG->reqFeatherAngleOfAttack1),-info.featherAoAHingeLimit,info.featherAoAHingeLimit,info.numPoints);
	} else if (m_numGeneration > 0) {
		perturbBestResult(&(m_currentBestCPG->reqWingFlappingAngle),&(info.birdCPG->reqWingFlappingAngle), -info.wingFlapHingeLimit,info.wingFlapHingeLimit,info.numPoints);
		perturbBestResult(&(m_currentBestCPG->reqFeatherAngleOfAttack1),&(info.birdCPG->reqFeatherAngleOfAttack1), -info.wingFlapHingeLimit,info.wingFlapHingeLimit,info.numPoints);
	}
	
	info.birdCPG->birdId = info.birdId;
	m_birdCPGs.push_back(info.birdCPG);

	m_currentBirdMetricDetails = new MetricDetails();
	m_currentBirdMetricDetails->birdId = info.birdId;
	m_bigbird = new BigBird(m_ownerWorld, info);
}

void BirdOptimizer::fillWithRandomNumbers(btAlignedObjectArray<btScalar>* arrScalar, btScalar minValue, btScalar maxValue, int numPoints)
{
	for (int ii = 0 ; ii < numPoints; ++ii) {
		arrScalar->push_back(minValue + (((double)rand())/RAND_MAX)*(maxValue-minValue));
	}
}

void BirdOptimizer::perturbBestResult(btAlignedObjectArray<btScalar>* arrBase, btAlignedObjectArray<btScalar>* arrChanged, btScalar minValue, btScalar maxValue, int numPoints)
{
	arrChanged->clear();
	arrChanged->copyFromArray(*arrBase);
	double mult = 0.0;
	for (int ii = 0 ; ii < arrChanged->size(); ++ii) {
		if (rand() < RAND_MAX/2)
			mult = 1.0;
		arrChanged->at(ii) += (minValue + (((double)rand())/RAND_MAX)*(maxValue-minValue))*mult;
		if (arrChanged->at(ii) > maxValue)
			arrChanged->at(ii) = maxValue;
		if (arrChanged->at(ii) < minValue)
			arrChanged->at(ii) = minValue;
	}
}

void BirdOptimizer::pretick(btScalar dt) {
	m_time += dt;
	
	if (m_bigbird) {
		m_bigbird->fillMetricDetails(m_currentBirdMetricDetails);
		m_bigbird->pretick(dt);

		if (m_bigbird->getTime() >= 15) {
			removeBigBird();
			spawnBigBird(btVector3(0,0,0));
		}

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

//need to make a proper evaluation function
//get the best CPG function, delete the rest of CPGs and clear the m_birdCPGs array
void BirdOptimizer::evaluateCurrentGenerationBirds() {
	if (m_birdCPGs.size()) {
		//choose the best CPG
		m_currentBestCPG = m_birdCPGs.at(0);
	}
	for (int ii = 0; ii < m_birdCPGs.size(); ii++) {
		if (m_birdCPGs.at(ii) == m_currentBestCPG) continue;
		delete m_birdCPGs.at(ii);
	}
	m_birdCPGs.clear();

	//this is just in case this gets called at some point not correctly
	if (!m_currentBestCPG) {
		m_currentBestCPG = new CPG();
		m_currentBestCPG->birdId = 0;
		fillWithRandomNumbers(&(m_currentBestCPG->reqWingFlappingAngle),0,0,180);
		fillWithRandomNumbers(&(m_currentBestCPG->reqFeatherAngleOfAttack1),0,0,180);
	}
}