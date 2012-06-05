#include "BirdOptimizer.h"
#include "../proto/proto_helper.h"

BirdOptimizer::BirdOptimizer(btDynamicsWorld* ownersWorld, int numBirds) 
: m_ownerWorld(ownersWorld), m_numBirdsPerGeneration(numBirds) {
	
	m_time = 0;
	m_numGeneration = -1;
	m_giveBirdThisId = 0;

	//Pointers.
	m_bigbird = 0;
	m_currentTrajectoryData = 0;
	m_currentBestInfo = 0;
	m_birdLocalParam = 0;
	m_birdData = 0;
	
	spawnBigBird(btVector3(0,0,0));
}

BirdOptimizer::~BirdOptimizer() {
	removeBigBird();
	for (int ii = 0 ; ii < m_birdTrajectoryData.size(); ii++) {
		delete m_birdTrajectoryData[ii];
	}

	for (int ii = 0; ii < m_birdInfos.size(); ii++) {
		if (m_birdInfos[ii] == m_currentBestInfo) continue;
		delete m_birdInfos[ii];
	}
	if (m_currentBestInfo)
		delete m_currentBestInfo;
}

void BirdOptimizer::spawnBigBird(const btVector3& startOffset)
{
	if ((m_giveBirdThisId % m_numBirdsPerGeneration) == 0) {
		m_numGeneration++;
		if (m_numGeneration > 0) {
			evaluateCurrentGenerationBirds();
		}
	}

	m_birdLocalParam = new BigBirdLocalParams();
	
	m_birdLocalParam->birdId = m_giveBirdThisId++;

	m_birdLocalParam->startTransform.setIdentity();
	m_birdLocalParam->startTransform.setOrigin(startOffset);

	m_birdLocalParam->hoistTransform.setIdentity();
	m_birdLocalParam->hoistTransform.setOrigin(startOffset);

	m_birdData = new proto::BigBirdConstructionData();

	m_birdData->set_hoistanglexy(30.f);
	m_birdData->set_hoistanglezxy(90.f);

	m_birdData->set_pelvishalflength(0.5f);
	m_birdData->set_winghalflength(0.5f);
	m_birdData->set_hoistmass(0.0f);
	m_birdData->set_pelvismass(2.0f);
	m_birdData->set_wingmass(0.5f);
	make_Vector3d(btVector3(0.f, 0.f, 0.f), m_birdData->mutable_pelvisrelpostoattachwing());
	make_Vector3d(btVector3(0.f, 0.f, 0.f), m_birdData->mutable_featherrelpostoattachfeather());
	m_birdData->set_wingflaphingelimit(90.f);
	m_birdData->set_featheraoahingelimit(90.f);
	m_birdData->set_featheraoamotormaximpulse(1000.0f);
	m_birdData->set_wingflapmotormaximpulse(1000.0f);

	m_birdData->set_randseed((unsigned int)time(NULL));
	srand(m_birdData->randseed());

	int numPoints = 180;

	if(m_numGeneration == 0) {
		fillWithRandomNumbers(m_birdData, numPoints);
	} else if (m_numGeneration > 0) {
		perturbBestResult(*m_currentBestInfo,m_birdData);
	}
	
	m_currentTrajectoryData = new proto::TrajectoryData();
	m_bigbird = new BigBird(m_ownerWorld, *m_birdLocalParam,*m_birdData);
}

void BirdOptimizer::fillWithRandomNumbers(proto::BigBirdConstructionData* info, int numPoints)
{
	for (int ii = 0 ; ii < numPoints; ++ii) {
		proto::WingbeatSample* sample = info->mutable_wingbeatdata()->add_sample();
		sample->set_wing(-info->wingflaphingelimit() + (((double)rand())/RAND_MAX)*(2*info->wingflaphingelimit()));
		sample->set_feather(-info->featheraoahingelimit() + (((double)rand())/RAND_MAX)*(2*info->featheraoahingelimit()));
	}
}

void BirdOptimizer::perturbBestResult(const proto::BigBirdConstructionData& bestInfo, proto::BigBirdConstructionData* info)
{
	info->CopyFrom(bestInfo);
	double mult = 0.0;
	for (int ii = 0 ; ii < info->wingbeatdata().sample_size(); ++ii) {
		if (rand() < RAND_MAX/2)
			mult = 1.0;
		proto::WingbeatSample* sample = info->mutable_wingbeatdata()->add_sample();
		float noise = 0.f;
		sample->set_wing(sample->wing() + noise);
		noise = 0.f;
		sample->set_feather(sample->feather() + noise);

		if (sample->wing() > info->wingflaphingelimit())
			sample->set_wing(info->wingflaphingelimit());
		if (sample->wing() < -info->wingflaphingelimit())
			sample->set_wing(-info->wingflaphingelimit());
		if (sample->feather() > info->featheraoahingelimit())
			sample->set_feather(info->featheraoahingelimit());
		if (sample->feather() < -info->featheraoahingelimit())
			sample->set_feather(-info->featheraoahingelimit());
	}
}

void BirdOptimizer::pretick(btScalar dt) {
	m_time += dt;
	
	if (m_bigbird) {
		m_bigbird->getCurrentTrajectory(m_currentTrajectoryData->add_sample());
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
		delete m_birdLocalParam;
		m_bigbird = 0;
		m_birdTrajectoryData.push_back(m_currentTrajectoryData);
		m_birdInfos.push_back(m_birdData);
		m_currentTrajectoryData = 0;
		m_birdLocalParam = 0;
		m_birdData = 0;
	}
}

//need to make a proper evaluation function
//get the best CPG function, delete the rest of CPGs and clear the m_birdCPGs array
void BirdOptimizer::evaluateCurrentGenerationBirds() {
	if (m_birdInfos.size()) {
		//choose the best CPG
		m_currentBestInfo = m_birdInfos[0];
	}
	for (int ii = 0; ii < m_birdInfos.size(); ii++) {
		if (m_birdInfos[ii] == m_currentBestInfo) continue;
		delete m_birdInfos[ii];
	}
	m_birdInfos.clear();

	//this is just in case this gets called at some point not correctly
	if (!m_currentBestInfo) {
		m_currentBestInfo = new proto::BigBirdConstructionData();
		fillWithRandomNumbers(m_currentBestInfo, 180);
	}
}