/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
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

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"

#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"



#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "ArtificialBirds.h"


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{
	enum
	{
		BODYPART_PELVIS = 0,

		BODYPART_TAIL,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_PELVIS_TAIL,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	btSoftBodyWorldInfo& m_softWorldInfo;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
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

	btSoftBody* localCreateSoftBody(btScalar mass, const btTransform& startTransform) {
		
		btSoftBody* body = btSoftBodyHelpers::CreatePatch(
			m_softWorldInfo,
			btVector3(0,0,0),
			btVector3(5,0,0),
			btVector3(0,5,0),
			btVector3(5,5,0),
			10,
			10,
			0,
			false);

		
		//btSoftBody* body = btSoftBodyHelpers::CreateEllipsoid(m_softWorldInfo, btVector3(0,0,0), btVector3(1,1,1), 10); 

		body->setTotalMass(mass);

		btSoftBody::Material* mat = body->appendMaterial();
		mat->m_kLST = btScalar(1.);
		mat->m_kAST = btScalar(1.);
		mat->m_flags -= btSoftBody::fMaterial::DebugDraw;
		body->generateBendingConstraints(2,mat);

		body->m_cfg.kDG = 0.005;
		body->m_cfg.kLF = 0.003;
		body->m_cfg.kMT = 1.;

		body->generateClusters(0);

		body->getCollisionShape()->setMargin(0.5);
		
		body->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
		//body->addForce(btVector3(0,2,0));
		body->transform(btTransform(startTransform));


		((btSoftRigidDynamicsWorld*)m_ownerWorld)->addSoftBody(body);
		
		return body;
	}

	btSoftBody* hoistInit(btTransform& startPosition, btRigidBody* body) {

	btCapsuleShape* capsule = (btCapsuleShape*)body->getCollisionShape();
	btScalar rad;
	btVector3 pos;
	capsule->getBoundingSphere(pos, rad);

	btScalar s = capsule->getHalfHeight();
    btScalar z = 2*rad;

	btSoftBody* arm = 0; {
		const btVector3	x[] = {
			// fixed
			btVector3(-2*z, z, 0),
			btVector3(-1*z, z, 0),
			btVector3(+1*z, z, 0),
			btVector3(+2*z, z, 0),

			// movable
			btVector3(-1*s, 0, 0),
			btVector3(+1*s, 0, 0),
		};

		const btScalar m[] = {0,0,0,0,1,1};

		btSoftBody* psb=new btSoftBody(&m_softWorldInfo, 6, x, m);

		btSoftBody::Material* mat = psb->appendMaterial();
		mat->m_kAST = 1;
		mat->m_kLST = 1;
		mat->m_flags -= btSoftBody::fMaterial::DebugDraw;

		
		//psb->appendLink(4,5, mat);

		psb->appendLink(4,1, mat);
		psb->appendLink(4,2, mat);
		psb->appendLink(5,1, mat);
		psb->appendLink(5,2, mat);

		psb->appendLink(4,0, mat);
		psb->appendLink(4,3, mat);
		psb->appendLink(5,0, mat);
		psb->appendLink(5,3, mat);

		psb->generateClusters(0);

		arm = psb;
	}

	
	((btSoftRigidDynamicsWorld*)m_ownerWorld)->addSoftBody(arm);
	arm->transform(startPosition);
	arm->appendAnchor(4, body, btVector3(0, s, 0));
	arm->appendAnchor(5, body, btVector3(0, -s, 0));

	return arm;
	}

	btSoftBody* getFeather(btRigidBody* rigidBody) {
		btCapsuleShape* collisionShape = (btCapsuleShape*)rigidBody->getCollisionShape();
		btVector3 center;
		btScalar radius;
		collisionShape->getBoundingSphere(center, radius);
		btScalar w = collisionShape->getHalfHeight();
		btScalar h = 0.5;

		btSoftBody* feather = 0; {
			const btVector3	x[] = {
				btVector3(0, 0, 0),
				btVector3(w, 0, 0),

				btVector3(0, h, 0),
				btVector3(w, h, 0),

				btVector3(0, 2*h, 0),
				btVector3(w, 2*h, 0),
			};

			const btScalar m[] = {0.1,0.1,0.1,0.1,0.1,0.1};

			btSoftBody* psb=new btSoftBody(&m_softWorldInfo, 6, x, m);

			btSoftBody::Material* mat = psb->appendMaterial();
			mat->m_kAST = 1;
			mat->m_kLST = 1;

			psb->appendLink(2,3, mat);

			psb->appendLink(2,0, mat);
			psb->appendLink(2,1, mat);

			psb->appendLink(3,0, mat);
			psb->appendLink(3,1, mat);


			psb->appendLink(4,2, mat);
			psb->appendLink(4,3, mat);

			psb->appendLink(5,2, mat);
			psb->appendLink(5,3, mat);

			psb->generateClusters(0);

			psb->appendFace(0, 1, 2);
			psb->appendFace(0, 1, 3);

			psb->appendFace(2, 3, 4);
			psb->appendFace(2, 3, 5);

			psb->m_cfg.aeromodel = btSoftBody::eAeroModel::F_TwoSidedLiftDrag;
			psb->m_cfg.kDG = 0.04;
			psb->m_cfg.kLF = 3;
			psb->m_cfg.kMT = 0.3;
			//psb->setPose(false,true);

			psb->setWindVelocity(btVector3(0, 20, 0));


			feather = psb;
		}

		((btSoftRigidDynamicsWorld*)m_ownerWorld)->addSoftBody(feather);

		btTransform trs;
		trs.setIdentity();
		trs.setOrigin(center);
		trs.getBasis().setEulerZYX(M_PI_2, M_PI_2, 0);
		feather->transform(trs);
		feather->appendAnchor(0, rigidBody, btVector3(0, w, 0));
		feather->appendAnchor(1, rigidBody, btVector3(0, -w, 0));
		feather->setPose(false,true);
		return feather;
	}

	void getRigidFeather(btRigidBody* rigidBody, btTransform& trs) {
		btCapsuleShape* collisionShape = (btCapsuleShape*)rigidBody->getCollisionShape();
		btVector3 center;
		btScalar radius;
		collisionShape->getBoundingSphere(center, radius);
		btScalar w = collisionShape->getHalfHeight();

		btTransform featherTransform;
		featherTransform.setIdentity();
		featherTransform.setOrigin(trs.getOrigin() + btVector3(0, w, 0));

		btRigidBody* feather_arm = localCreateRigidBody(0.1, featherTransform, new btCapsuleShape(0.01, w*2));

		btTransform localA; localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(0,-w,0));
		btTransform localB; localB.getBasis().setEulerZYX(M_PI_2,0,0); localB.setOrigin(btVector3(0,0,0));
		btHingeConstraint* hingeC =  new btHingeConstraint(*feather_arm, *rigidBody, localA, localB);

		hingeC->setLimit(-SIMD_PI/8, SIMD_PI/8);

		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(hingeC);

		btSoftBody* feather = 0; {
			const btVector3	x[] = {
				btVector3(0, 0, 0),
				btVector3(-w, 0.7*w, 0),
				btVector3(w, 0.7*w, 0),
				btVector3(0, w, 0),

			};

			const btScalar m[] = {0.1,0.1,0.1,0.1};

			btSoftBody* psb=new btSoftBody(&m_softWorldInfo, 4, x, m);

			btSoftBody::Material* mat = psb->appendMaterial();
			mat->m_kAST = 0.6;
			mat->m_kLST = 0.6;

			psb->appendLink(0,1, mat);
			psb->appendLink(0,2, mat);
			psb->appendLink(0,3, mat);
			psb->appendLink(1,3, mat);
			psb->appendLink(2,3, mat);
			psb->generateClusters(0);

			psb->appendFace(0, 1, 3);
			psb->appendFace(0, 2, 3);

			psb->m_cfg.aeromodel = btSoftBody::eAeroModel::F_TwoSidedLiftDrag;
			psb->m_cfg.kDG = 0.04;
			psb->m_cfg.kLF = 0.05;
			psb->m_cfg.kMT = 0.9;
			psb->m_cfg.kCHR = 0.9;
			psb->setPose(false,true);

			//psb->setWindVelocity(btVector3(55.0, 0, 0));
			feather = psb;
		}

		((btSoftRigidDynamicsWorld*)m_ownerWorld)->addSoftBody(feather);

		feather->transform(featherTransform);
		feather->appendAnchor(0, feather_arm, btVector3(0, 0, 0));

	}

public:
	RagDoll (btDynamicsWorld* ownerWorld, btSoftBodyWorldInfo& softWorldInfo, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld), m_softWorldInfo (softWorldInfo)
	{
		// Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_TAIL] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(0.1), offset*transform, m_shapes[BODYPART_PELVIS]);

		//btSoftBody* hoistBody = hoistInit(offset*transform, m_bodies[BODYPART_PELVIS]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_TAIL] = localCreateRigidBody(btScalar(0.1), offset*transform, m_shapes[BODYPART_TAIL]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(0.1), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(0.1), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);
		//getRigidFeather(m_bodies[BODYPART_LEFT_LOWER_ARM], offset*transform);
		


		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(0.1), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(0.1), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);
		//getRigidFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], offset*transform);
		//getRigidFeather(m_bodies[BODYPART_LEFT_LOWER_ARM]);
		//getRigidFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM]);

		getFeather(m_bodies[BODYPART_RIGHT_UPPER_ARM]);
	    getFeather(m_bodies[BODYPART_LEFT_UPPER_ARM]);
		getFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM]);
		getFeather(m_bodies[BODYPART_LEFT_LOWER_ARM]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.5, 0.90);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity();
		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.))); 
		localB.setIdentity();
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-.10), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_TAIL], localA, localB);
		coneC->setLimit(M_PI_4/4.0, M_PI_4/4.0, 0);
		m_joints[JOINT_PELVIS_TAIL] = coneC;
		//coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_TAIL], true);

		// LEFT_SHOULDER
		localA.setIdentity();localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(0, SIMD_PI/9, 0);
		//coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		// RIGHT_SHOULDER
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(0, SIMD_PI/9, 0);
		//coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		// LEFT_ELBOW
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
		hingeC->setLimit(0, 0);
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);

		// RIGHT_ELBOW
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
		hingeC->setLimit(0, 0);
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		//hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);




		// Try to add soft body.
		/*
		btTransform trs;
		trs.setIdentity();
		trs.setOrigin(btVector3(0,0,20));
		btSoftBody* psb = localCreateSoftBody(btScalar(0.1), trs);
		psb->appendAnchor(0, m_bodies[BODYPART_LEFT_UPPER_ARM]);
		*/


		/*
		btTransform offset;
		offset.setIdentity();
		offset.setOrigin(lerp(arm->m_nodes[4].m_x, arm->m_nodes[5].m_x, 0.5));
		offset.setRotation(btQuaternion(btVector3(0,0,1), SIMD_2_PI/4));
		btCollisionShape* shape = new btCapsuleShape(s/5, 2*s);
		btRigidBody* body = localCreateRigidBody(btScalar(0.4), offset, shape);

		offset.setIdentity();
		*/

		


	}

	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}
};




void ArtificialBirds::initPhysics()
{
	// Setup the basic world

	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_softBodyWorldInfo.air_density	= 5;
	m_softBodyWorldInfo.water_density = 0;
	m_softBodyWorldInfo.water_offset = 0;
	m_softBodyWorldInfo.water_normal = btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
	
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



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

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	m_softBodyWorldInfo.m_sparsesdf.Reset();

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	spawnRagdoll(startOffset);

	// Spawn another ragdoll
	//startOffset.setValue(-1,0.5,0);
	//spawnRagdoll(startOffset);

	clientResetScene();		
}

void ArtificialBirds::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, m_softBodyWorldInfo, startOffset);
	m_ragdolls.push_back(ragDoll);
}	

void ArtificialBirds::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();


	}
	
	m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void ArtificialBirds::renderme() {
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();

	int debugMode = m_dynamicsWorld->getDebugDrawer()? m_dynamicsWorld->getDebugDrawer()->getDebugMode() : -1;

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	btIDebugDraw*	sdraw = softWorld ->getDebugDrawer();


	for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
	{
		btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
		}
	}
	DemoApplication::renderme();
}

void ArtificialBirds::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void ArtificialBirds::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		btVector3 startOffset(0,2,0);
		spawnRagdoll(startOffset);
		break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}



void	ArtificialBirds::exitPhysics()
{

	int i;

	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
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





