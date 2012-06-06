#include "BirdDemo.h"
#include "Bird.h"
#include "proto_helper.h"

const int kNumGenerationsToLoad = 5;

BirdDemo::BirdDemo(btDynamicsWorld* ownerWorld) : m_ownerWorld(ownerWorld) {
	m_numBirds = kNumGenerationsToLoad;
	m_time = 0;
	m_cameraViewer = 0;
	m_hasMass = true;
	m_randomData = true;
	m_optimizedData = new proto::BirdOptimizerData();
	m_generationChooser = new int[kNumGenerationsToLoad];

	loadData();

	createBirds();
}

BirdDemo::~BirdDemo() {
	removeAllBirds();
	if (m_optimizedData)
		delete m_optimizedData;
}

void BirdDemo::toggleBirdWingbeatData() {
	m_randomData = !m_randomData;
	
	loadData();

	createBirds();
}

void BirdDemo::loadData() {
	std::string filename = "..\\experiment_data\\8_seconds_unif\\bird_data.pbdata";
	if (!m_randomData)
		filename = "..\\experiment_data\\8_seconds_flight_2\\bird_data.pbdata";

	std::ifstream ifs(filename, std::ios::in | std::ios::binary);
	assert(ifs.is_open());

	m_optimizedData->Clear();

	if (!m_optimizedData->ParseFromIstream(&ifs)) {
		std::cerr << "Cannot parse protobuf!" << std::endl;
	}

	ifs.close();

	//Hack. Need to find a better way to choose the generations.
	for (int ii = 0 ; ii < kNumGenerationsToLoad - 1; ++ii) {
		m_generationChooser[ii] = ii*(m_optimizedData->result_size()/kNumGenerationsToLoad);
	}
	m_generationChooser[kNumGenerationsToLoad-1] = m_optimizedData->result_size() - 1;
}

void BirdDemo::toggleMasslessBirds() {
	m_hasMass = !m_hasMass;
	createBirds();
}


void BirdDemo::createBirds() {
	removeAllBirds();
	for (int ii = 0; ii < m_numBirds; ++ii) {
		Bird* bird = new Bird();
		createBird(bird,m_generationChooser[ii]);
		m_birdArray.push_back(bird);
	}
}

void BirdDemo::createBird(Bird* bird, int numGen) {
	btVector3 startOffset = btVector3(0.f,5.f,numGen*1.f);
	BigBirdLocalParams* birdLocalParams= new BigBirdLocalParams();

	birdLocalParams->birdId = numGen;

	birdLocalParams->startTransform.setIdentity();
	birdLocalParams->startTransform.setOrigin(startOffset);

	birdLocalParams->hoistTransform.setIdentity();
	birdLocalParams->hoistTransform.setOrigin(startOffset);

	proto::BigBirdConstructionData* birdConstData = new proto::BigBirdConstructionData();
	if (numGen < m_optimizedData->result().size())
		birdConstData->CopyFrom(m_optimizedData->result(numGen).bird());

	bird->m_birdConstData = birdConstData;
	bird->m_birdLocalParams = birdLocalParams;

	if (!m_hasMass)
		bird->m_birdConstData->set_pelvismass(0.f);

	bird->m_bigbird = new BigBird(m_ownerWorld,*(bird->m_birdLocalParams),*(bird->m_birdConstData));
}

void BirdDemo::pretick(btScalar dt) {
	m_time += dt;

	for (int ii = 0; ii < m_birdArray.size(); ++ii) {
		m_birdArray[ii]->m_bigbird->pretick(dt);
	}

}

void BirdDemo::removeBird(int index) {
	if (m_birdArray.size() > index) {
		delete m_birdArray[index]->m_bigbird;
		delete m_birdArray[index]->m_birdConstData;
		delete m_birdArray[index]->m_birdLocalParams;
		m_birdArray.remove(m_birdArray[index]);
	}
}

void BirdDemo::removeAllBirds() {
	for (int ii = 0; ii < m_birdArray.size(); ++ii) {
		delete m_birdArray[ii]->m_bigbird;
		delete m_birdArray[ii]->m_birdConstData;
		delete m_birdArray[ii]->m_birdLocalParams;
	}
	m_birdArray.clear();
}
