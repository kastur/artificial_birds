#include "ArtificialBirds.h"
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "BigBird.h"
#include "BigFeather.h"
#include "time.h"

const btScalar kGravity = -9.80;
const int kSolverNumIterations = 100;

void pickingPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	ArtificialBirdsDemoApp* app = (ArtificialBirdsDemoApp*)world->getWorldUserInfo();

	for (int ii = 0; ii < app->getBirds().size(); ++ii) {
		app->getBirds()[ii]->pretick(timeStep);
	}
}

void ArtificialBirdsDemoApp::initPhysics()
{
	// Setup the basic world
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);
	m_dynamicsWorld->getSolverInfo().m_numIterations = kSolverNumIterations;
	m_dynamicsWorld->setGravity(btVector3(0,kGravity,0));

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}
	spawnBigBird(btVector3(0, 0, 0));

	clientResetScene();		
}

void ArtificialBirdsDemoApp::spawnBigBird(const btVector3& startOffset)
{
	BigBirdConstructionInfo info;
	
	info.birdId = m_bigbirds.size();
	//info.convert << "birdInfo" << info.birdId << ".txt";
	//info.file.open(info.convert.str());
	
	info.startTransform.setIdentity();
	info.startTransform.setOrigin(startOffset);

	info.hoistTransform.setIdentity();
	info.hoistTransform.setOrigin(startOffset);
	info.hoistAngle = 240.f;

	info.pelvisHalfLength = 1.0f;
	info.wingHalfLength = 0.6f;
	info.hoistMass = 0.0f;
	info.pelvisMass = 5.0f;
	info.wingMass = 1.0f;
	info.pelvisRelPosToAttachWing = btVector3(0.f, 0.f, 0.f);
	info.featherRelPosToAttachFeather = btVector3(0.f, 0.f, 0.f);
	info.wingFlapHingeLimit = 90.f;
	info.featherAoAHingeLimit = 90.f;
	info.featherAoAMotorMaxImpulse = 10.0f;
	info.wingFlapMotorMaxImpulse = 10.0f;
	info.wingFlapFrequency = 1.5f;


	info.randSeed = (unsigned int)time(NULL);
	srand(info.randSeed);

	info.numPoints = 200;
	info.reqWingFlappingAngle = new btScalar[info.numPoints];
	info.reqFeatherAngleOfAttack1 = new btScalar[info.numPoints];
	info.reqFeatherAngleOfAttack2 = new btScalar[info.numPoints];
	info.reqFeatherAngleOfAttack3 = new btScalar[info.numPoints];

	fillWithRandomNumbers(info.reqWingFlappingAngle,-info.wingFlapHingeLimit,info.wingFlapHingeLimit,info.numPoints);
	fillWithRandomNumbers(info.reqFeatherAngleOfAttack1,-info.featherAoAHingeLimit,info.featherAoAHingeLimit,info.numPoints);
	fillWithRandomNumbers(info.reqFeatherAngleOfAttack2,-info.featherAoAHingeLimit,info.featherAoAHingeLimit,info.numPoints);
	fillWithRandomNumbers(info.reqFeatherAngleOfAttack3,-info.featherAoAHingeLimit,info.featherAoAHingeLimit,info.numPoints);

	BigBird* bigbird = new BigBird(m_dynamicsWorld, info);
	m_bigbirds.push_back(bigbird);
}


void ArtificialBirdsDemoApp::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld) {
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	m_cameraTargetPosition = m_bigbirds[0]->getPosition();

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void ArtificialBirdsDemoApp::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void ArtificialBirdsDemoApp::keyboardCallback(unsigned char key, int x, int y) {
	switch (key) {
	case '!':
		break;
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}
}

void ArtificialBirdsDemoApp::exitPhysics()
{

	int i;

	for (i=0;i<m_bigbirds.size();i++) {
		BigBird* bird = m_bigbirds[i];
		delete bird;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	
	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;	
}

void ArtificialBirdsDemoApp::fillWithRandomNumbers(btScalar* arrScalar, btScalar minValue, btScalar maxValue, int numPoints)
{
	for (int ii = 0 ; ii < numPoints; ++ii) {
		arrScalar[ii] = minValue + (((double)rand())/RAND_MAX)*(maxValue-minValue);
	}
}

void ArtificialBirdsDemoApp::removeBird(int birdId) {
	BigBird* bird = m_bigbirds[birdId];
	delete bird;
	for (int ii = birdId + 1; ii < m_bigbirds.size(); ++ii) {
		m_bigbirds[ii - 1] = m_bigbirds[ii];
	}
}