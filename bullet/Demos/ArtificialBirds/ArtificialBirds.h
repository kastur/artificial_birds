/*
Bullet Continuous Collision Detection and Physics Library
ArtificialBirdsDemoApp
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef ARTIFICIAL_BIRDS_H__
#define ARTIFICIAL_BIRDS_H__

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class ArtificialBirdsDemoApp : public GlutDemoApplication {

	btAlignedObjectArray<class BigBird*> m_bigbirds;
	btAlignedObjectArray<class BigFeather*> m_bigfeathers;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	class BirdOptimizer* m_birdOpt;

	void removeBigBird(int id);
public:
	void initPhysics();

	void exitPhysics();

	virtual ~ArtificialBirdsDemoApp()
	{
		exitPhysics();
	}

	void spawnBigBird(const btVector3& startOffset);
	void spawnBigFeather(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		ArtificialBirdsDemoApp* demo = new ArtificialBirdsDemoApp();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	btAlignedObjectArray<class BigFeather*> getFeathers() { return m_bigfeathers; }
	btAlignedObjectArray<class BigBird*> getBirds() { return m_bigbirds; }
	BirdOptimizer* getBirdOptimizer() {return m_birdOpt;}
	
};


#endif
