#include "BigBird.h"

#define CONSTRAINT_DEBUG_SIZE 1.0f

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif


btRigidBody* BigBird::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}

BigBird::BigBird (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld)	{
	t = 0;

	// Setup the geometry
	m_shapes[BODYPART_PELVIS]			= new btCapsuleShape(btScalar(0.15), btScalar(1));

	m_shapes[BODYPART_RIGHT_UPPER_ARM]	= new btCapsuleShape(btScalar(0.06), btScalar(0.35));
	m_shapes[BODYPART_LEFT_UPPER_ARM]	= new btCapsuleShape(btScalar(0.06), btScalar(0.35));

	m_shapes[BODYPART_RIGHT_LOWER_ARM]	= new btCapsuleShape(btScalar(0.06), btScalar(0.57));
	m_shapes[BODYPART_LEFT_LOWER_ARM]	= new btCapsuleShape(btScalar(0.06), btScalar(0.57));

	m_shapes[BODYPART_LEFT_WRIST]		= new btCapsuleShape(btScalar(0.06), btScalar(0.48));
	m_shapes[BODYPART_RIGHT_WRIST]		= new btCapsuleShape(btScalar(0.06), btScalar(0.48));

	btScalar mass[BODYPART_COUNT];
	mass[BODYPART_PELVIS]			= 0.0;
	mass[BODYPART_RIGHT_UPPER_ARM]	= 0.2;
	mass[BODYPART_LEFT_UPPER_ARM]	= 0.2;
	mass[BODYPART_RIGHT_LOWER_ARM]	= 0.1;
	mass[BODYPART_LEFT_LOWER_ARM]	= 0.1;
	mass[BODYPART_RIGHT_WRIST]		= 0.1;
	mass[BODYPART_LEFT_WRIST]		= 0.1;
	
	
	// Setup all the rigid bodies
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(0.3), btScalar(0.)));
	m_bodies[BODYPART_PELVIS] = localCreateRigidBody(mass[BODYPART_PELVIS], offset*transform, m_shapes[BODYPART_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(mass[BODYPART_LEFT_UPPER_ARM], offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(mass[BODYPART_LEFT_LOWER_ARM], offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-1.05), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	m_bodies[BODYPART_LEFT_WRIST] = localCreateRigidBody(mass[BODYPART_LEFT_WRIST], offset*transform, m_shapes[BODYPART_LEFT_WRIST]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(mass[BODYPART_RIGHT_UPPER_ARM], offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(mass[BODYPART_RIGHT_LOWER_ARM], offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(1.05), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	m_bodies[BODYPART_RIGHT_WRIST] = localCreateRigidBody(mass[BODYPART_RIGHT_WRIST], offset*transform, m_shapes[BODYPART_RIGHT_WRIST]);

	// Setup some damping on the m_bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05, 0.85);
		m_bodies[i]->setDeactivationTime(900.0);
		m_bodies[i]->setSleepingThresholds(0, 0);
	}

	// Now setup the constraints
	btConeTwistConstraint* coneC;

	btTransform localA, localB;

	// LEFT_SHOULDER
	localA.setIdentity();localB.setIdentity();
	localA.setOrigin(btVector3(btScalar(-0.18), btScalar(0.34), btScalar(0.)));
	localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.19), btScalar(0.)));
	localA.getBasis().setEulerZYX(0,btRadians(180+35),btRadians(25));
	localB.getBasis().setEulerZYX(0,0,btRadians(90));
	coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
	coneC->setLimit(btRadians(15), btRadians(35), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_LEFT_SHOULDER] = coneC;
	m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
		
	// RIGHT_SHOULDER
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(btScalar(0.18), btScalar(0.34), btScalar(0.)));
	localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.19), btScalar(0.)));
	localA.getBasis().setEulerZYX(0,btRadians(-35),btRadians(-25));
	localB.getBasis().setEulerZYX(0,0,btRadians(90));
	coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
	coneC->setLimit(btRadians(15), btRadians(35), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_RIGHT_SHOULDER] = coneC;
	m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
	
	// LEFT_ELBOW
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(btScalar(0.), btScalar(0.19), btScalar(0.)));
	localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.29), btScalar(0.)));
	localA.getBasis().setEulerZYX(0,btRadians(35),btRadians(55)); 
	localB.getBasis().setEulerZYX(0, 0, 0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
	coneC->setLimit(0, 0,0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_LEFT_ELBOW] = coneC;
	m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);

	// RIGHT_ELBOW
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(btScalar(0.), btScalar(0.19), btScalar(0.)));
	localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.29), btScalar(0.)));
	localA.getBasis().setEulerZYX(0,btRadians(35),btRadians(55)); 
	localB.getBasis().setEulerZYX(0, btRadians(90), 0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
	coneC->setLimit(0, 0, 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_RIGHT_ELBOW] = coneC;
	m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);

	// LEFT_WRIST
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(btScalar(0.), btScalar(0.29), btScalar(0.)));
	localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.25), btScalar(0.)));
	localA.getBasis().setEulerZYX(0,btRadians(0),btRadians(-55)); 
	localB.getBasis().setEulerZYX(0,btRadians(0),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_LOWER_ARM], *m_bodies[BODYPART_LEFT_WRIST], localA, localB);
	coneC->setLimit(0, 0, 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_LEFT_WRIST] = coneC;
	m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_WRIST], true);
		
	// RIGHT_WRIST
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(btScalar(0.), btScalar(0.29), btScalar(0.)));
	localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.25), btScalar(0.)));
	localA.getBasis().setEulerZYX(0,btRadians(0),btRadians(-55)); 
	localB.getBasis().setEulerZYX(0,btRadians(0),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_LOWER_ARM], *m_bodies[BODYPART_RIGHT_WRIST], localA, localB);
	coneC->setLimit(0, 0, 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_RIGHT_WRIST] = coneC;
	m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_WRIST], true);

	// Attach feathers
	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-20),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-20),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}


	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0.1, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-30),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0.1, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-30),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}


	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, -0.25, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(30),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_LOWER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, -0.25, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(30),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_LOWER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}


	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, -0.1, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(20),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_LOWER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, -0.1, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(20),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_LOWER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0.1, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(10),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_LOWER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0.1, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(10),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_LOWER_ARM], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, -0.2, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-30),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_WRIST], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, -0.2, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-30),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_WRIST], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-40),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_WRIST], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-40),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_WRIST], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0.2, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-50),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_LEFT_WRIST], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}

	{
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, btVector3(-0.35, +1.45, +0.00));
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	localA.setIdentity(); localB.setIdentity();
	localA.setOrigin(btVector3(0.06, 0.2, 0));
	localB.setOrigin(btVector3(-0.4, 0, 0));
	localA.getBasis().setEulerZYX(btRadians(90),0,0); 
	localB.getBasis().setEulerZYX(0,btRadians(-50),0); 
	coneC =  new btConeTwistConstraint(*m_bodies[BODYPART_RIGHT_WRIST], *feather, localA, localB);
	coneC->setLimit(btRadians(2), btRadians(2), 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
	}
	
	

	// Enable motors

	((btConeTwistConstraint*)m_joints[JOINT_LEFT_SHOULDER])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_SHOULDER])->setMaxMotorImpulse(200);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMaxMotorImpulse(200);

	((btConeTwistConstraint*)m_joints[JOINT_LEFT_ELBOW])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_ELBOW])->enableMotor(true);
}

BigBird::~BigBird() {
	int i;

	// Remove all constraints
	for ( i = 0; i < JOINT_COUNT; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for ( i = 0; i < BODYPART_COUNT; ++i) {
		m_ownerWorld->removeRigidBody(m_bodies[i]);
		delete m_bodies[i]->getMotionState();
		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}

	for (int ii = 0; ii < m_feathers.size(); ++ii) {
		delete m_feathers[ii];
	}

	for (int ii = 0; ii < m_featherjoints.size(); ++ii) {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_featherjoints[ii];
	}
}

void BigBird::pretick (btScalar dt) {
	t += dt;  // keep track of time.

	// Compute aerodynamic forces for each feather.
	for (int ii = 0; ii < m_feathers.size(); ++ii) {
		m_feathers[ii]->pretick(dt);
	}

	// Set motors.
	btScalar freq = 1.5;

	// Wingbeat
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_SHOULDER])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(-35*btSin(t*SIMD_2_PI*freq))));
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(35*btSin(t*SIMD_2_PI*freq))));

	((btConeTwistConstraint*)m_joints[JOINT_LEFT_ELBOW])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(30)));
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_ELBOW])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(30)));

	((btConeTwistConstraint*)m_joints[JOINT_LEFT_WRIST])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(30)));
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_WRIST])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(30)));

}