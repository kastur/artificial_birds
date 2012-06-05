#include "BirdOptimizer.h"
#include "../proto/proto_helper.h"

//#define RAPID_TESTING

const int kNumSamplesInWingbeat = 18;
const float kPerturbScaler = 0.1f;
#ifdef RAPID_TESTING
const int kBirdLifeTime = 10;
const int kNumBirdsPerGeneration = 2;
#else
const int kBirdLifeTime = 6;
const int kNumBirdsPerGeneration = 10;
#endif

BirdOptimizer::BirdOptimizer(btDynamicsWorld* ownersWorld) 
	: m_ownerWorld(ownersWorld) {
	
	m_time = 0;
	m_numGeneration = -1;
	m_giveBirdThisId = 0;

	//Pointers.
	m_bigbird = 0;
	m_currentTrajectoryData = 0;
	m_currentBestInfo = 0;
	m_currentBirdLocalParam = 0;
	m_currentBirdData = 0;
	m_rand_eng.seed();
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
	if ((m_giveBirdThisId % kNumBirdsPerGeneration) == 0) {
		m_numGeneration++;
		if (m_numGeneration > 0) {
			evaluateCurrentGenerationBirds();
		}
	}

	m_currentBirdLocalParam = new BigBirdLocalParams();
	
	m_currentBirdLocalParam->birdId = m_giveBirdThisId++;

	m_currentBirdLocalParam->startTransform.setIdentity();
	m_currentBirdLocalParam->startTransform.setOrigin(startOffset);

	m_currentBirdLocalParam->hoistTransform.setIdentity();
	m_currentBirdLocalParam->hoistTransform.setOrigin(startOffset);

	m_currentBirdData = new proto::BigBirdConstructionData();

	m_currentBirdData->set_hoistanglexy(30.f);
	m_currentBirdData->set_hoistanglezxy(90.f);

	m_currentBirdData->set_pelvishalflength(0.5f);
	m_currentBirdData->set_winghalflength(0.5f);
	m_currentBirdData->set_hoistmass(0.0f);
	m_currentBirdData->set_pelvismass(2.0f);
	m_currentBirdData->set_wingmass(0.5f);
	make_Vector3d(btVector3(0.f, 0.f, 0.f), m_currentBirdData->mutable_pelvisrelpostoattachwing());
	make_Vector3d(btVector3(0.f, 0.f, 0.f), m_currentBirdData->mutable_featherrelpostoattachfeather());
	m_currentBirdData->set_wingflaphingelimit(70.f);
	m_currentBirdData->set_featheraoahingelimit(35.f);
	m_currentBirdData->set_featheraoamotormaximpulse(1000.0f);
	m_currentBirdData->set_wingflapmotormaximpulse(1000.0f);

	m_currentBirdData->set_randseed((unsigned int)time(NULL));
	srand(m_currentBirdData->randseed());

	int numPoints = kNumSamplesInWingbeat;

	if(m_numGeneration == 0) {
		fillWithRandomNumbers(m_currentBirdData, numPoints);
	} else if (m_numGeneration > 0 && (m_giveBirdThisId % kNumBirdsPerGeneration) == 0) {
		m_currentBirdData->CopyFrom(*m_currentBestInfo);
	} else if (m_numGeneration > 0) {
		perturbBestResult(*m_currentBestInfo,m_currentBirdData);
	}
	
	//std::cout << m_currentBirdData->wingbeatdata().DebugString() << std::endl;

	m_currentTrajectoryData = new proto::TrajectoryData();
	m_bigbird = new BigBird(m_ownerWorld, *m_currentBirdLocalParam,*m_currentBirdData);
}

void BirdOptimizer::fillWithRandomNumbers(proto::BigBirdConstructionData* info, int numPoints)
{
	for (int ii = 0 ; ii < numPoints; ++ii) {
		btScalar req_angle = info->wingflaphingelimit() * btSin((float)ii/numPoints * SIMD_2_PI);
		btScalar feather_angle = info->featheraoahingelimit() * btCos((float)ii/numPoints * SIMD_2_PI);
		proto::WingbeatSample* sample = info->mutable_wingbeatdata()->add_sample();
		sample->set_wing(req_angle);
		sample->set_feather(feather_angle);
		sample->set_wing(-info->wingflaphingelimit() + (((double)rand())/RAND_MAX)*(2*info->wingflaphingelimit()));
		sample->set_feather(-info->featheraoahingelimit() + (((double)rand())/RAND_MAX)*(2*info->featheraoahingelimit()));
	}
}

void BirdOptimizer::perturbBestResult(const proto::BigBirdConstructionData& bestInfo, proto::BigBirdConstructionData* info)
{
	info->CopyFrom(bestInfo);
	double mult = 0.0;
	int infoSize = info->wingbeatdata().sample_size();
	for (int ii = 0 ; ii < infoSize; ++ii) {
		proto::WingbeatSample* sample = info->mutable_wingbeatdata()->mutable_sample(ii);

		float noise = kPerturbScaler * info->wingflaphingelimit() * m_rand_dist(m_rand_eng);
		sample->set_wing(sample->wing() + noise);

		noise = kPerturbScaler * info->featheraoahingelimit() * m_rand_dist(m_rand_eng);
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
		proto::TrajectorySample* sample = m_currentTrajectoryData->add_sample();
		m_bigbird->getCurrentTrajectory(sample);

		//std::cout << sample->ShortDebugString() << std::endl;

		m_bigbird->pretick(dt);

		if (m_bigbird->getTime() >= kBirdLifeTime) {
			removeBigBird();
			spawnBigBird(btVector3(0,0,0));
		}

	}
}

void BirdOptimizer::removeBigBird() {
	if (m_bigbird) {
		delete m_bigbird;
		delete m_currentBirdLocalParam;
		m_bigbird = 0;
		m_birdTrajectoryData.push_back(m_currentTrajectoryData);
		m_birdInfos.push_back(m_currentBirdData);
		assert(m_birdInfos.size() == m_birdTrajectoryData.size());
		m_currentTrajectoryData = 0;
		m_currentBirdLocalParam = 0;
		m_currentBirdData = 0;
	}


}

float calculateEnergy(proto::TrajectoryData* trajectoryData) {
	float energy = 0;

	//TODO: check why we skip first second
	std::cout << "sample_size: " << trajectoryData->sample_size() << std::endl;
	for (int ii = 60; ii < trajectoryData->sample_size(); ++ii) {
		const proto::TrajectorySample& sample = trajectoryData->sample(ii);
		energy += sample.rightfeatherimpulse()*sample.rightfeatherimpulse();
		energy += sample.leftfeatherimpulse()*sample.leftfeatherimpulse();
		energy += sample.rightwingimpulse()*sample.rightwingimpulse();
		energy += sample.leftwingimpulse()*sample.leftwingimpulse();
	}
	return energy;
}

//need to make a proper evaluation function
//get the best CPG function, delete the rest of CPGs and clear the m_birdCPGs array
void BirdOptimizer::evaluateCurrentGenerationBirds() {

	//choose the best CPG
	float minEnergy = FLT_MAX;
	int minIndex = -1;
	for (int ii = 0; ii < m_birdTrajectoryData.size(); ++ii) {
		float energy = calculateEnergy(m_birdTrajectoryData[ii]);
		std::cout << "energy: " << energy << std::endl;
		if (energy < minEnergy)
		{
			minEnergy = energy;
			minIndex = ii;
		}
	}
	std::cout << "min_energy: " << minEnergy << " index: " << minIndex << std::endl;
	m_currentBestInfo = m_birdInfos[minIndex];

	// Save bird configuration.

	proto::BirdOptimizerResult* result = m_result_data.add_result();
	result->set_cum_energy(minEnergy);
	result->mutable_bird()->CopyFrom(*m_birdInfos[minIndex]);

	if ((m_numGeneration-1 % 20) == 0) {
		{
		std::ofstream ofs("C:\\Users\\k\\bird_data.pbdata");
		ofs << m_result_data.SerializeAsString();
		}
		{
		std::ofstream ofs("C:\\Users\\k\\bird_data.txt");
		ofs << m_result_data.DebugString();
		}
	}


	for (int ii = 0; ii < m_birdTrajectoryData.size(); ++ii) {
		delete m_birdTrajectoryData[ii];
	}
	m_birdTrajectoryData.clear();

	for (int ii = 0; ii < m_birdInfos.size(); ii++) {
		if (m_birdInfos[ii] == m_currentBestInfo) continue;
		delete m_birdInfos[ii];
	}
	m_birdInfos.clear();

	//this is just in case this gets called at some point not correctly
	if (!m_currentBestInfo) {
		m_currentBestInfo = new proto::BigBirdConstructionData();
		fillWithRandomNumbers(m_currentBestInfo, kNumSamplesInWingbeat);
	}
}