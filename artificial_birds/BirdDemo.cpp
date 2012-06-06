#include "BirdDemo.h"
#include "Bird.h"
#include "proto_helper.h"
#include <vector>

const int kNumGenerationsToLoad = 5;

BirdDemo::BirdDemo(btDynamicsWorld* ownerWorld) : m_ownerWorld(ownerWorld) {
	m_numBirds = kNumGenerationsToLoad;
	m_time = 0;
	m_cameraViewer = 0;
	m_singleBird = false;
	m_randomData = true;
	m_optimizedData = new proto::BirdOptimizerData();
	m_generationChooser.reserve(kNumGenerationsToLoad);

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
	std::string filename = "..\\experiment_data\\8_seconds_unif_2\\bird_data.pbdata";
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
	float multiplier = (float)m_optimizedData->result_size()/(kNumGenerationsToLoad-1);
	m_generationChooser.clear();
	for (int ii = 0 ; ii < kNumGenerationsToLoad - 1; ++ii) {
		m_generationChooser.insert(m_generationChooser.begin() + ii,ii*multiplier);
	}
	m_generationChooser.insert(m_generationChooser.begin() + (kNumGenerationsToLoad - 1), m_optimizedData->result_size() - 1);
}

void BirdDemo::toggleMasslessBirds() {
	for (int ii = 0; ii < m_birdArray.size(); ++ii) {
		m_birdArray[ii]->m_bigbird->toggleHoist();
	}
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
	btVector3 startOffset = btVector3(0.f,5.f,numGen*0.5f);
	BigBirdLocalParams* birdLocalParams= new BigBirdLocalParams();

	birdLocalParams->birdId = numGen;

	birdLocalParams->startTransform.setIdentity();
	birdLocalParams->startTransform.setOrigin(startOffset);

	birdLocalParams->hoistTransform.setIdentity();
	birdLocalParams->hoistTransform.setOrigin(startOffset);

	proto::BigBirdConstructionData* birdConstData = new proto::BigBirdConstructionData();
	if (0 <= numGen && numGen < m_optimizedData->result().size())
		birdConstData->CopyFrom(m_optimizedData->result(numGen).bird());

	bird->m_birdConstData = birdConstData;
	bird->m_birdLocalParams = birdLocalParams;

	bird->m_bigbird = new BigBird(m_ownerWorld,*(bird->m_birdLocalParams),*(bird->m_birdConstData));
}

void BirdDemo::pretick(btScalar dt) {
	m_time += dt;

	if (!m_singleBird) {
		for (int ii = 0; ii < m_birdArray.size(); ++ii) {
			m_birdArray[ii]->m_bigbird->pretick(dt);
		}
	} else {
		if (0 < m_cameraViewer && m_cameraViewer < m_birdArray.size() )
			m_birdArray[m_cameraViewer]->m_bigbird->pretick(dt);
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

void BirdDemo::toggleSingleBirdDemo() {
	m_singleBird = !m_singleBird;
}