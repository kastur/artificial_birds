#include <iostream>

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
		
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}

BigBird::BigBird (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld)	{
	t = 0;

	const btScalar upper_arm_h = 1.0;
	const btScalar lower_arm_h = 0.57;
	const btScalar wrist_h = 0.48;
	// Setup the geometry
	m_shapes[BODYPART_PELVIS]			= new btCapsuleShape(btScalar(0.10), btScalar(1.00));

	m_shapes[BODYPART_RIGHT_UPPER_ARM]	= new btCapsuleShape(btScalar(0.06), upper_arm_h);
	m_shapes[BODYPART_LEFT_UPPER_ARM]	= new btCapsuleShape(btScalar(0.06), upper_arm_h);

	m_shapes[BODYPART_RIGHT_LOWER_ARM]	= new btCapsuleShape(btScalar(0.06), lower_arm_h);
	m_shapes[BODYPART_LEFT_LOWER_ARM]	= new btCapsuleShape(btScalar(0.06), lower_arm_h);

	m_shapes[BODYPART_LEFT_WRIST]		= new btCapsuleShape(btScalar(0.06), wrist_h);
	m_shapes[BODYPART_RIGHT_WRIST]		= new btCapsuleShape(btScalar(0.06), wrist_h);

	btScalar mass[BODYPART_COUNT];
	mass[BODYPART_PELVIS]			= 2.0;
	mass[BODYPART_RIGHT_UPPER_ARM]	= 1.0;
	mass[BODYPART_LEFT_UPPER_ARM]	= 1.0;
	mass[BODYPART_RIGHT_LOWER_ARM]	= 0.1;
	mass[BODYPART_LEFT_LOWER_ARM]	= 0.1;
	mass[BODYPART_RIGHT_WRIST]		= 0.0;
	mass[BODYPART_LEFT_WRIST]		= 0.0;

	// Setup all the rigid bodies
	btTransform offset;
	offset.setIdentity();
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.00), btScalar(1.45), btScalar(0.00)));
	transform.getBasis().setEulerZYX(btRadians(90), 0, 0);
	m_bodies[BODYPART_PELVIS] = localCreateRigidBody(mass[BODYPART_PELVIS], offset*transform, m_shapes[BODYPART_PELVIS]);


	// UPPER_ARMS
	{
		btScalar xpos = upper_arm_h/2 + 0.2;

		transform.setIdentity();
		transform.setOrigin(btVector3(0.00 - xpos, 1.45, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(90));
		m_bodies[BODYPART_LEFT_UPPER_ARM] =
			localCreateRigidBody(
				mass[BODYPART_LEFT_UPPER_ARM],
				offset * transform,
				m_shapes[BODYPART_LEFT_UPPER_ARM]);
	
		transform.setIdentity();
		transform.setOrigin(btVector3(0.00 + xpos, 1.45, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(-90));
		m_bodies[BODYPART_RIGHT_UPPER_ARM] =
			localCreateRigidBody(
				mass[BODYPART_RIGHT_UPPER_ARM],
				offset * transform,
				m_shapes[BODYPART_RIGHT_UPPER_ARM]);
	}
	
	// LOWER_ARMS
	{
		btScalar xpos = upper_arm_h + lower_arm_h/2 + 0.4;

		transform.setIdentity();
		transform.setOrigin(btVector3(0.00 - xpos, 1.45, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(90));
		m_bodies[BODYPART_LEFT_LOWER_ARM] =
			localCreateRigidBody(
				mass[BODYPART_LEFT_LOWER_ARM],
				offset * transform,
				m_shapes[BODYPART_LEFT_LOWER_ARM]);
	
		transform.setIdentity();
		transform.setOrigin(btVector3(0.00 + xpos, 1.45, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(-90));
		m_bodies[BODYPART_RIGHT_LOWER_ARM] =
			localCreateRigidBody(
				mass[BODYPART_RIGHT_LOWER_ARM],
				offset * transform,
				m_shapes[BODYPART_RIGHT_LOWER_ARM]);
	}
	
	// WRISTS
	{
		btScalar xpos = upper_arm_h + lower_arm_h + wrist_h/2 + 0.6;

		transform.setIdentity();
		transform.setOrigin(btVector3(0.00 - xpos, 1.45, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(90));
		m_bodies[BODYPART_LEFT_WRIST] =
			localCreateRigidBody(
				mass[BODYPART_LEFT_WRIST],
				offset * transform,
				m_shapes[BODYPART_LEFT_WRIST]);
	
		transform.setIdentity();
		transform.setOrigin(btVector3(0.00 + xpos, 1.45, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(-90));
		m_bodies[BODYPART_RIGHT_WRIST] =
			localCreateRigidBody(
				mass[BODYPART_RIGHT_WRIST],
				offset * transform,
				m_shapes[BODYPART_RIGHT_WRIST]);
	}

	// Setup some damping on the m_bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05, 0.85);
		m_bodies[i]->setDeactivationTime(900.0);
		m_bodies[i]->setSleepingThresholds(0, 0);
	}
	
	// Now setup the constraints
	btHingeConstraint* hingeC;
	btTransform localA;
	btTransform localB;

	// LEFT_SHOULDER
	hingeC =
		new btHingeConstraint(
			*m_bodies[BODYPART_LEFT_UPPER_ARM],
			*m_bodies[BODYPART_PELVIS],
			btVector3(+0.00, -upper_arm_h/2, +0.00),
			btVector3(-0.18, +0.34, +0.00),
			btVector3(0, 0, 1),
			btVector3(0, 1, 0)
			);
	hingeC->setLimit(btRadians(90), btRadians(180-20));
	m_joints[JOINT_LEFT_SHOULDER] = hingeC;
	m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
	
	
	// RIGHT_SHOULDER
	hingeC =
		new btHingeConstraint(
		*m_bodies[BODYPART_PELVIS],	
			*m_bodies[BODYPART_RIGHT_UPPER_ARM],
			btVector3(+0.18, +0.34, +0.00),
			btVector3(+0.00, -upper_arm_h/2, +0.00),
			btVector3(0, 1, 0),
			btVector3(0, 0, 1)
			);
	hingeC->setLimit(btRadians(90), btRadians(180-20));
	m_joints[JOINT_RIGHT_SHOULDER] = hingeC;
	m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
	
	/*
	// LEFT_ELBOW
	hingeC =
		new btHingeConstraint(
			*m_bodies[BODYPART_LEFT_LOWER_ARM],
			*m_bodies[BODYPART_LEFT_UPPER_ARM],
			btVector3(+0.00, -lower_arm_h/2, +0.00),
			btVector3(+0.00, +upper_arm_h/2, +0.00),
			btVector3(0, 0, 1),
			btVector3(1, 0, 0)
			);
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	hingeC->setLimit(btRadians(0), btRadians(30));
	m_joints[JOINT_LEFT_ELBOW] = hingeC;
	m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
	
	
	// RIGHT_ELBOW
	hingeC =
		new btHingeConstraint(
			*m_bodies[BODYPART_RIGHT_LOWER_ARM],
			*m_bodies[BODYPART_RIGHT_UPPER_ARM],
			btVector3(+0.00, -lower_arm_h/2, +0.00),
			btVector3(+0.00, +upper_arm_h/2, +0.00),
			btVector3(0, 0, 1),
			btVector3(1, 0, 0));
	hingeC->setLimit(btRadians(0), btRadians(30));
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_RIGHT_ELBOW] = hingeC;
	m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	*/
	
	// Attach feathers
	
	addFeather(m_bodies[BODYPART_RIGHT_UPPER_ARM], btVector3(0,-0.15,0), btRadians(90), btRadians(90), btRadians(0), btRadians(0));
	addFeather(m_bodies[BODYPART_LEFT_UPPER_ARM],  btVector3(0,-0.15,0), btRadians(90), btRadians(90), btRadians(0), btRadians(0));

	addFeather(m_bodies[BODYPART_RIGHT_UPPER_ARM], btVector3(0,+0.0,0), btRadians(90), btRadians(90), btRadians(0), btRadians(10));
	addFeather(m_bodies[BODYPART_LEFT_UPPER_ARM],  btVector3(0,+0.0,0), btRadians(90), btRadians(90), btRadians(0), btRadians(10));

	addFeather(m_bodies[BODYPART_RIGHT_UPPER_ARM], btVector3(0,+0.15,0), btRadians(90), btRadians(90), btRadians(0), btRadians(10));
	addFeather(m_bodies[BODYPART_LEFT_UPPER_ARM],  btVector3(0,+0.15,0), btRadians(90), btRadians(90), btRadians(0), btRadians(10));
	

	

	addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,-0.2,0), btRadians(90), btRadians(0), btRadians(0), btRadians(10));
	addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,-0.2,0), btRadians(90), btRadians(0), btRadians(0), btRadians(10));
	
	addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,+0.0,0), btRadians(90), btRadians(0), btRadians(0), btRadians(20));
	addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,+0.0,0), btRadians(90), btRadians(0), btRadians(0), btRadians(20));

	addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,+0.2,0), btRadians(90), btRadians(0), btRadians(0), btRadians(30));
	addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,+0.2,0), btRadians(90), btRadians(0), btRadians(0), btRadians(30));

	//addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,-0.1,0), btRadians(90), btRadians(180), btRadians(55));
	//addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,-0.1,0), btRadians(90), btRadians(180), btRadians(-25));

	//addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,-0.0,0), btRadians(90), btRadians(180), btRadians(45));
	//addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,-0.0,0), btRadians(90), btRadians(180), btRadians(-30));

	//addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,-0.1,0), btRadians(90), btRadians(180), btRadians(35));
	//addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,-0.1,0), btRadians(90), btRadians(180), btRadians(-35));

	//addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,-0.2,0), btRadians(90), btRadians(180), btRadians(25));
	//addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,-0.2,0), btRadians(90), btRadians(180), btRadians(-40));

	// Enable motors
	
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER])->enableMotor(true);
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER])->setMaxMotorImpulse(100);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->enableMotor(true);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMaxMotorImpulse(100);

	/*
	((btHingeConstraint*)m_joints[JOINT_LEFT_ELBOW])->enableMotor(true);
	((btHingeConstraint*)m_joints[JOINT_LEFT_ELBOW])->setMaxMotorImpulse(5000);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_ELBOW])->enableMotor(true);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_ELBOW])->setMaxMotorImpulse(5000);
	*/
	/*
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_ELBOW])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_ELBOW])->setMaxMotorImpulse(1000);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_ELBOW])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_ELBOW])->setMaxMotorImpulse(1000);

	((btConeTwistConstraint*)m_joints[JOINT_LEFT_WRIST])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_WRIST])->setMaxMotorImpulse(1000);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_WRIST])->enableMotor(true);
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_WRIST])->setMaxMotorImpulse(1000);
	*/

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
	btScalar freq = 1;

	// Wingbeat
	btScalar req_angle = 35*btSin(t*SIMD_2_PI*freq) + 35;
	//std::cout << req_angle << std::endl;
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(90  + req_angle), dt);
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER] )->setMotorTarget(btRadians(90  + req_angle), dt);

	//((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER])->setMotorTarget(btRadians(90), dt);
	//((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(90), dt);

	//((btHingeConstraint*)m_joints[JOINT_LEFT_ELBOW])->setMotorTarget(btRadians(0), dt);
	//((btHingeConstraint*)m_joints[JOINT_RIGHT_ELBOW])->setMotorTarget(btRadians(0), dt);


	// Arm bend.
	/*
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_SHOULDER ])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(+35)));
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(-35)));

	
	((btConeTwistConstraint*)m_joints[JOINT_LEFT_ELBOW])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(-30)));
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_ELBOW])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(-30)));

	((btConeTwistConstraint*)m_joints[JOINT_LEFT_WRIST])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(-30)));
	((btConeTwistConstraint*)m_joints[JOINT_RIGHT_WRIST])->setMotorTargetInConstraintSpace(btQuaternion(btVector3(0,1,0), btRadians(-30)));
	*/
}

void BigBird::addFeather(btRigidBody* rb, const btVector3& relPos, btScalar rbAngleX, btScalar rbAngleY, btScalar featherAngle, btScalar featherGive) {
	btVector3 feather_pos(rb->getWorldTransform().getOrigin() + relPos + btVector3(-0.5, 0, 0));
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos);
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();

	btTransform localA;
	btTransform localB;
	localA.setIdentity();
	localB.setIdentity();
	localA.setOrigin(relPos);
	localB.setOrigin(btVector3(-0.5, 0, 0));
	localA.getBasis().setEulerZYX(rbAngleX, rbAngleY, btRadians(0)); 
	localB.getBasis().setEulerZYX(btRadians(0), featherAngle, btRadians(0)); 

	btConeTwistConstraint* coneC = new btConeTwistConstraint(*rb, *feather, localA, localB);
	coneC->setLimit(btRadians(0), btRadians(featherGive), btRadians(5));
	coneC->setDbgDrawSize(0);
	m_featherjoints.push_back(coneC);
	m_ownerWorld->addConstraint(coneC, true);
}

void BigBird::applyImpulse() {
	//btVector3 axis = m_bodies[BODYPART_PELVIS]->getCenterOfMassTransform().getRotation().getAxis();
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(0, 2, 0));
	transform.getBasis().setEulerZYX(btRadians(90), 0, 0);

	m_bodies[BODYPART_PELVIS]->setLinearVelocity(btVector3(0,0,0));
	m_bodies[BODYPART_PELVIS]->setAngularVelocity(btVector3(0,0,0));

	btVector3 impulse = m_bodies[BODYPART_PELVIS]->getWorldTransform().getBasis() * btVector3(0, 10.0*10, 0);

	m_bodies[BODYPART_PELVIS]->applyImpulse(impulse, btVector3(0,0,0));
	
	/*
	for (int ii = 0; ii < m_feathers.size(); ++ii) {
		m_feathers[ii]->applyImpulse();
	}
	*/
}