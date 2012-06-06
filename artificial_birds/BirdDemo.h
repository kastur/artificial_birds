#ifndef BIRD_DEMO_H__
#define BIRD_DEMO_H__

#include "btBulletDynamicsCommon.h"
#include "LinearMath\btAlignedObjectArray.h"
#include "BigBirdLocalParams.h"
#include "../proto/proto.pb.h"
#include "BigBird.h"
#include "Bird.h"
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

class BirdDemo {
public:
	BirdDemo(btDynamicsWorld* ownersWorld);
	void pretick(btScalar dt);
	void toggleMasslessBirds();
	void toggleBirdWingbeatData();
	void toggleSingleBirdDemo();
	void createBirds();
	void cycleThroughBirdViews() {
		m_cameraViewer++;
		if (m_cameraViewer > m_birdArray.size())
			m_cameraViewer = 0;
	}
	btVector3 getPosition() { 
		if (0 < m_cameraViewer && m_cameraViewer < m_birdArray.size())
			return m_birdArray[m_cameraViewer]->m_bigbird->getPosition();
		return btVector3(0.f,10.f,0.f);
		}
	~BirdDemo();

protected:
	void createBird(Bird* bird, int numGen);
	void loadData();
	void removeBird(int index);
	void removeAllBirds();

private:
	proto::BirdOptimizerData* m_optimizedData;
	btAlignedObjectArray<Bird*> m_birdArray;
	btDynamicsWorld* m_ownerWorld;
	btScalar m_time;
	std::vector<int> m_generationChooser;
	int m_numBirds;
	int m_cameraViewer;
	bool m_singleBird;
	bool m_randomData;
};



#endif