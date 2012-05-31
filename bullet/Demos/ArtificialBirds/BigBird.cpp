#include <iostream>

#include "BigBird.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f

const btScalar kWingPelvisAttachPosition = 0.00f;
const btScalar kWingFeatherAttachPosition = 0.00f;
const btScalar kMaxFeatherImpulse = 1000.0f;
const btScalar kMaxLimbImpulse = 1000.0f;
const btScalar kWingbeatFrequency = 3.5f;

const btScalar kFeatherLimit = 40.0f;
const btScalar kShoulderLimit = 70.0f;

btRigidBody* BigBird::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
	btVector3 localInertia(0,0,0);
	if (mass != 0.0f)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);	
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	m_ownerWorld->addRigidBody(body);
	return body;
}

BigBird::BigBird (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld)	{
	t = 0;

	m_feather_angle = 0.0f;

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
	mass[BODYPART_RIGHT_UPPER_ARM]	= 0.5;
	mass[BODYPART_LEFT_UPPER_ARM]	= 0.5;
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
		btScalar xpos = upper_arm_h/2 + 0.2f;

		transform.setIdentity();
		transform.setOrigin(btVector3(0.00f - xpos, 1.45f, 0.00));
		transform.getBasis().setEulerZYX(0, 0, btRadians(90));
		m_bodies[BODYPART_LEFT_UPPER_ARM] =
			localCreateRigidBody(
				mass[BODYPART_LEFT_UPPER_ARM],
				offset * transform,
				m_shapes[BODYPART_LEFT_UPPER_ARM]);
	
		transform.setIdentity();
		transform.setOrigin(btVector3(0.00f + xpos, 1.45f, 0.00));
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
		btScalar xpos = upper_arm_h + lower_arm_h + wrist_h/2 + 0.6f;

		transform.setIdentity();
		transform.setOrigin(btVector3(0.00f - xpos, 1.45f, 0.00));
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
		m_bodies[i]->setDamping(0.00, 0.80);
		m_bodies[i]->setDeactivationTime(900.0);
		m_bodies[i]->setSleepingThresholds(0.0, 0.0);
	}
	// Make sure pelvis is damped!
	m_bodies[BODYPART_PELVIS]->setDamping(0.00, 0.90);
	
	// Now setup the constraints
	btHingeConstraint* hingeC;
	btTransform localA;
	btTransform localB;

	{ // LEFT_SHOULDER
		hingeC = new btHingeConstraint(
				*m_bodies[BODYPART_LEFT_UPPER_ARM],	
				*m_bodies[BODYPART_PELVIS],	
				btVector3(+0.00, -upper_arm_h/2, +0.00),
				btVector3(0.00, kWingPelvisAttachPosition, +0.00),
				btVector3(0, 0, -1),
				btVector3(0, 1, 0)
				);
		hingeC->setLimit(btRadians(90-kShoulderLimit), btRadians(90+kShoulderLimit));
		hingeC->enableMotor(true);
		hingeC->setMaxMotorImpulse(kMaxLimbImpulse);
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER] = hingeC;
	}
	
	{ // RIGHT_SHOULDER
		hingeC = new btHingeConstraint(
				*m_bodies[BODYPART_RIGHT_UPPER_ARM],	
				*m_bodies[BODYPART_PELVIS],	
				btVector3(+0.00, -upper_arm_h/2, +0.00),
				btVector3(0.00, kWingPelvisAttachPosition, +0.00),
				btVector3(0, 0, 1),
				btVector3(0, 1, 0)
				);
		hingeC->setLimit(btRadians(90-kShoulderLimit), btRadians(90+kShoulderLimit));
		hingeC->enableMotor(true);
		hingeC->setMaxMotorImpulse(kMaxLimbImpulse);
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER] = hingeC;
	}

	{  // LEFT FEATHER
		btRigidBody* rb = m_bodies[BODYPART_LEFT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers.push_back(bigfeather);
		btRigidBody* feather = bigfeather->getFeatherBody();

		hingeC = new btHingeConstraint(
				*rb,
				*feather,
				btVector3(0, 0, 0),
				btVector3(kWingFeatherAttachPosition, 0, 0),
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(true);
		hingeC->setMaxMotorImpulse(kMaxFeatherImpulse);
		hingeC->setLimit(btRadians(90-kFeatherLimit), btRadians(90+kFeatherLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER_FEATHER] = hingeC;
	}

	{  // RIGHT FEATHER
		btRigidBody* rb = m_bodies[BODYPART_RIGHT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers.push_back(bigfeather);
		btRigidBody* feather = bigfeather->getFeatherBody();

		hingeC = new btHingeConstraint(
				*rb,
				*feather,
				btVector3(0, 0, 0),
				btVector3(kWingFeatherAttachPosition, 0, 0),
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(true);
		hingeC->setMaxMotorImpulse(kMaxFeatherImpulse);
		hingeC->setLimit(btRadians(90-kFeatherLimit), btRadians(90+kFeatherLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER_FEATHER] = hingeC;
	}
	
	/*
	// LEFT_ELBOW
	hingeC =
		new btHingeConstraint(
			*m_bodies[BODYPART_LEFT_LOWER_ARM],
			*m_bodies[BODYPART_LEFT_UPPER_ARM],
			btVector3(+0.00, -lower_arm_h/2, +0.00),
			btVector3(+0.00, +upper_arm_h/2, +0.00),
			btVector3(1, 0, 0),
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
			btVector3(1, 0, 0),
			btVector3(1, 0, 0));
	hingeC->setLimit(btRadians(0), btRadians(30));
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_joints[JOINT_RIGHT_ELBOW] = hingeC;
	m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
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
	// Keep track of time.
	t += dt;

	// Compute aerodynamic forces for each feather.
	for (int ii = 0; ii < m_feathers.size(); ++ii) {
		m_feathers[ii]->pretick(dt);
	}

	// Wing flapping
	btScalar req_angle = kShoulderLimit*btSin(t*SIMD_2_PI*kWingbeatFrequency);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(90  - req_angle), dt);
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER] )->setMotorTarget(btRadians(90  + req_angle), dt);

	
	// Feather angle of attack
	btScalar feather_angle = kFeatherLimit*btCos(t*SIMD_2_PI*kWingbeatFrequency);
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER_FEATHER ])->setMotorTarget(btRadians(90 + feather_angle), dt);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER_FEATHER])->setMotorTarget(btRadians(90 - feather_angle), dt);

}