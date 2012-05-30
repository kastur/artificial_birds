#include <iostream>

#include "BigBird.h"

#define CONSTRAINT_DEBUG_SIZE 0.0f

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

	impulse_pretick = btVector3(0,0,0);
	impulse_relpos = btVector3(0,0,0);
	
	const btScalar pelvis_h = 1.0;
	const btScalar upper_arm_h = 0.38;//.378
	const btScalar lower_arm_h = 0.57;//.57
	const btScalar wrist_h = 0.48;//.47
	// Setup the geometry
	m_shapes[BODYPART_PELVIS]			= new btCapsuleShape(btScalar(0.10), btScalar(pelvis_h));

	m_shapes[BODYPART_RIGHT_UPPER_ARM]	= new btCapsuleShape(btScalar(0.06), upper_arm_h);
	m_shapes[BODYPART_LEFT_UPPER_ARM]	= new btCapsuleShape(btScalar(0.06), upper_arm_h);

	m_shapes[BODYPART_RIGHT_LOWER_ARM]	= new btCapsuleShape(btScalar(0.06), lower_arm_h);
	m_shapes[BODYPART_LEFT_LOWER_ARM]	= new btCapsuleShape(btScalar(0.06), lower_arm_h);

	m_shapes[BODYPART_LEFT_WRIST]		= new btCapsuleShape(btScalar(0.06), wrist_h);
	m_shapes[BODYPART_RIGHT_WRIST]		= new btCapsuleShape(btScalar(0.06), wrist_h);

	btScalar mass[BODYPART_COUNT];
	mass[BODYPART_PELVIS]			= 2.00;
	mass[BODYPART_RIGHT_UPPER_ARM]	= 0.38;
	mass[BODYPART_LEFT_UPPER_ARM]	= 0.38;
	mass[BODYPART_RIGHT_LOWER_ARM]	= 0.57;
	mass[BODYPART_LEFT_LOWER_ARM]	= 0.57;
	mass[BODYPART_RIGHT_WRIST]		= 0.48;
	mass[BODYPART_LEFT_WRIST]		= 0.48;

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
		m_bodies[i]->setDamping(0.10, 0.85);
		m_bodies[i]->setDeactivationTime(900.0);
		m_bodies[i]->setSleepingThresholds(1.6, 2.5);
	}
	
	// Now setup the constraints
	btHingeConstraint* hingeC;
	btTransform localA;
	btTransform localB;

	//shoulders
	{
		// LEFT_SHOULDER
		hingeC =
			new btHingeConstraint(
				*m_bodies[BODYPART_LEFT_UPPER_ARM],
				*m_bodies[BODYPART_PELVIS],
				btVector3(+0.00, -upper_arm_h/2, +0.00),
				btVector3(-0.18, +0.14, +0.00),
				btVector3(0, 0, 1),
				btVector3(0, 1, 0)
				);
		//hingeC->setLimit(btRadians(90), btRadians(180-20));
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_joints[JOINT_LEFT_SHOULDER] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
	
	
		// RIGHT_SHOULDER
		hingeC =
			new btHingeConstraint(
			*m_bodies[BODYPART_PELVIS],	
				*m_bodies[BODYPART_RIGHT_UPPER_ARM],
				btVector3(+0.18, +0.14, +0.00),
				btVector3(+0.00, -upper_arm_h/2, +0.00),
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		//hingeC->setLimit(btRadians(90), btRadians(180-20));
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_joints[JOINT_RIGHT_SHOULDER] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
	}
	
	//elbows
	/*{
		// LEFT_ELBOW
		hingeC =
			new btHingeConstraint(
				*m_bodies[BODYPART_LEFT_UPPER_ARM],
				*m_bodies[BODYPART_LEFT_LOWER_ARM],
				btVector3(+0.00, +upper_arm_h/2, +0.00),
				btVector3(+0.00, -lower_arm_h/2, +0.00),
				btVector3(0, 0, 1),
				btVector3(0, 0, 1)
				);
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		hingeC->setLimit(btRadians(0), btRadians(15));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
	
	
		// RIGHT_ELBOW
		hingeC =
			new btHingeConstraint(
				*m_bodies[BODYPART_RIGHT_UPPER_ARM],
				*m_bodies[BODYPART_RIGHT_LOWER_ARM],
				btVector3(+0.00, +upper_arm_h/2, +0.00),
				btVector3(+0.00, -lower_arm_h/2, +0.00),
				btVector3(0, 0, 1),
				btVector3(0, 0, 1)
				);
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		hingeC->setLimit(btRadians(-15), btRadians(0));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}
	
	//wrists
	/*{
		// LEFT_WRIST
		hingeC =
			new btHingeConstraint(
				*m_bodies[BODYPART_LEFT_LOWER_ARM],
				*m_bodies[BODYPART_LEFT_WRIST],
				btVector3(+0.00, +lower_arm_h/2, +0.00),
				btVector3(+0.00, -wrist_h/2, +0.00),
				btVector3(0, 0, 1),
				btVector3(0, 0, 1)
				);
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		hingeC->setLimit(btRadians(0), btRadians(15));
		m_joints[JOINT_LEFT_WRIST] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_WRIST], true);
	
	
		// RIGHT_WRIST
		hingeC =
			new btHingeConstraint(
				*m_bodies[BODYPART_RIGHT_LOWER_ARM],
				*m_bodies[BODYPART_RIGHT_WRIST],
				btVector3(+0.00, +lower_arm_h/2, +0.00),
				btVector3(+0.00, -wrist_h/2, +0.00),
				btVector3(0, 0, 1),
				btVector3(0, 0, 1)
				);
		hingeC->setLimit(btRadians(-15), btRadians(0));
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_joints[JOINT_RIGHT_WRIST] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_WRIST], true);
	}/**/

	// Attach feathers
	btScalar numFeathers = 1;
	btScalar featherWidth = upper_arm_h/numFeathers;
	btScalar featherWidthHalf = featherWidth/2;
	for (btScalar pp = (-upper_arm_h/2 + featherWidthHalf); pp <= upper_arm_h/2; pp += featherWidth) {
		addFeather(m_bodies[BODYPART_RIGHT_UPPER_ARM], btVector3(0,pp,0), btRadians(90), btRadians(180+90+10), btRadians(0), btRadians(0), featherWidthHalf);
		addFeather(m_bodies[BODYPART_LEFT_UPPER_ARM],  btVector3(0,pp,0), btRadians(90), btRadians(180+90-10), btRadians(0), btRadians(0), featherWidthHalf);
	}

	numFeathers = 4;
	featherWidth = lower_arm_h/numFeathers;
	featherWidthHalf = featherWidth/2;
	for (btScalar pp = (-lower_arm_h/2 + featherWidthHalf); pp <= lower_arm_h/2; pp += featherWidth) {
		addFeather(m_bodies[BODYPART_RIGHT_LOWER_ARM], btVector3(0,pp,0), btRadians(90), btRadians(180+90+10), btRadians(0), btRadians(0),featherWidthHalf);
		addFeather(m_bodies[BODYPART_LEFT_LOWER_ARM],  btVector3(0,pp,0), btRadians(90), btRadians(180+90-10), btRadians(0), btRadians(0),featherWidthHalf);
	}

	numFeathers = 3;
	featherWidth = wrist_h/numFeathers;
	featherWidthHalf = featherWidth/2;
	for (btScalar pp = (-wrist_h/2 + featherWidthHalf); pp <= wrist_h/2; pp += featherWidth) {
		addFeather(m_bodies[BODYPART_RIGHT_WRIST], btVector3(0,pp,0), btRadians(90), btRadians(180+90+10), btRadians(0), btRadians(0),featherWidthHalf);
		addFeather(m_bodies[BODYPART_LEFT_WRIST],  btVector3(0,pp,0), btRadians(90), btRadians(180+90-10), btRadians(0), btRadians(0),featherWidthHalf);
	}

	//Tail
	numFeathers = 5;
	featherWidth = pelvis_h/numFeathers;
	featherWidthHalf = featherWidth/2;
	for (int ii = 0; ii < numFeathers; ii++)
	{
		btScalar pp = ii - numFeathers/2;
		addTailFeather(m_bodies[BODYPART_PELVIS], btVector3(pp*.1,-pelvis_h/2,0), btRadians(0), btRadians(90), btRadians(0), btRadians(0),.2);
	}

	// Enable motors
	motor_state = true;
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER])->enableMotor(motor_state);
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER])->setMaxMotorImpulse(500);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->enableMotor(motor_state);
	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMaxMotorImpulse(500);

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

	// Set motors.
	btScalar freq = 3;

	// Compute aerodynamic forces for each feather.
	bool aero_on = true;
	if ((btSin(t*SIMD_2_PI*freq) - btSin((t+dt)*SIMD_2_PI*freq)) > 0)
	{
		aero_on = false;
	}

	for (int ii = 0; ii < m_feathers.size(); ++ii) {
		//m_feathers[ii]->aero_on = aero_on;
		m_feathers[ii]->pretick(dt);
	}


	// Wingbeat
	btScalar req_angle = 35*btSin(t*SIMD_2_PI*freq) + 35;
	//std::cout << req_angle << std::endl;

	if (!motor_state)
		req_angle = 0;

	
	if (impulse_pretick.length() > 0.01) {
		m_bodies[BODYPART_PELVIS]->applyForce(impulse_pretick, impulse_relpos);
	}

	((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(90  + req_angle), dt);
	((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER] )->setMotorTarget(btRadians(90  + req_angle), dt);
	//((btHingeConstraint*)m_joints[JOINT_LEFT_ELBOW])->setMotorTarget(btRadians(90  + req_angle), dt);
	//((btHingeConstraint*)m_joints[JOINT_RIGHT_ELBOW])->setMotorTarget(btRadians(90  + req_angle), dt);

	btScalar left_angle = 90;
	btScalar right_angle = 90 ;
	//((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER])->setMotorTarget(btRadians(left_angle), dt);
	//((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(right_angle), dt);

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

void BigBird::addFeather(btRigidBody* rb, const btVector3& relPos, btScalar rbAngleX, btScalar rbAngleY, btScalar featherAngle, btScalar featherGive, btScalar featherWidthHalf) {
	btVector3 feather_pos = rb->getCenterOfMassPosition();
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb, featherWidthHalf,0.01,0.45);
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();
	
	btHingeConstraint* hingeC =
		new btHingeConstraint(
			*rb,
			*feather,
			relPos,
			btVector3(-0.5, 0, 0),
			btVector3(0, 1, 0),
			btVector3(0, 0, 1)
			);
	hingeC->setLimit(rbAngleY-btRadians(0), rbAngleY+btRadians(0));
	m_ownerWorld->addConstraint(hingeC, true);
	m_featherjoints.push_back(hingeC);
	
}

void BigBird::addTailFeather(btRigidBody* rb, const btVector3& relPos, btScalar rbAngleX, btScalar rbAngleY, btScalar featherAngle, btScalar featherGive, btScalar featherWidthHalf) {
	btVector3 feather_pos = rb->getCenterOfMassPosition();
	BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb, 0.3,0.01,0.05);
	m_feathers.push_back(bigfeather);
	btRigidBody* feather = bigfeather->getFeatherBody();

	btTransform trA;
	trA.setIdentity();
	trA.setOrigin(btVector3(0,0,0));
	trA.getBasis().setEulerZYX(btRadians(90),btRadians(0),btRadians(0));
	feather->setCenterOfMassTransform(trA);

	btHingeConstraint* hingeC =
		new btHingeConstraint(
			*rb,
			*feather,
			relPos,
			btVector3(0, 0, featherWidthHalf),
			btVector3(1, 0, 0),
			btVector3(1, 0, 0)
			);
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	hingeC->setLimit(rbAngleY-btRadians(15), rbAngleY+btRadians(0));
	m_ownerWorld->addConstraint(hingeC, true);
	m_featherjoints.push_back(hingeC);
	
}

void BigBird::applyImpulse() {
	
	btVector3 impulse = m_bodies[BODYPART_PELVIS]->getWorldTransform().getBasis() * btVector3(0, 80.0, 0);
	impulse_pretick = impulse;
	impulse_relpos = btVector3(0,0,0);
}

void BigBird::applyUpImpulse() {
	btVector3 impulse = m_bodies[BODYPART_PELVIS]->getWorldTransform().getBasis() * btVector3(0.0, 0.0, 100.0);
	impulse_pretick = impulse;
	impulse_relpos = btVector3(0, -0.5, 0); 
}

void BigBird::applyDownImpulse() {
	btVector3 impulse = m_bodies[BODYPART_PELVIS]->getWorldTransform().getBasis() * btVector3(0.0, 0.0, -20.0);
	impulse_pretick = impulse;
	impulse_relpos = btVector3(0, +0.5, 0);	
}

void BigBird::zeroImpulse() {
	impulse_pretick = btVector3(0,0,0);
}

void BigBird::applyFeatherImpulse() {
	for (int ii = 0; ii < m_feathers.size(); ++ii) {
		m_feathers[ii]->applyImpulse();
	}
}

void BigBird::toggleMotors() {
	motor_state = !motor_state;
}