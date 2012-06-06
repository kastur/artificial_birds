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
	void createBirds();
	void cycleThroughBirdViews() {
		m_cameraViewer++;
		if (m_cameraViewer > m_birdArray.size())
			m_cameraViewer = 0;
	}
	btVector3 getPosition() { 
		if (m_birdArray.size() > m_cameraViewer)
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
	int* m_generationChooser;
	int m_numBirds;
	int m_cameraViewer;
	bool m_hasMass;
	bool m_randomData;
};



#endif