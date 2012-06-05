#include "BigBird.h"
#include "../proto/proto_helper.h"

const bool kEnableMotors = true;

btRigidBody* BigBird::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
	btVector3 localInertia(0,0,0);
	if (mass != 0.0f)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);	
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	m_ownerWorld->addRigidBody(body);
	return body;
}

BigBird::BigBird(btDynamicsWorld* ownerWorld, const BigBirdLocalParams& local_info, const proto::BigBirdConstructionData& data) :
	m_ownerWorld(ownerWorld),
	m_local_info(local_info),
	m_data(data) {

	m_time = 0;
	m_time_steps = -1;

	m_hoist_shapes[HOIST_POINT_0] = new btCapsuleShape(0.1f, 0.1f);
	m_hoist_shapes[HOIST_POINT_1] = new btCapsuleShape(0.1f, 0.1f);
	m_hoist_shapes[HOIST_POINT_2] = new btCapsuleShape(0.1f, 0.1f);

	m_shapes[BODYPART_PELVIS] = new btCapsuleShape(0.1f, m_data.pelvishalflength() * 2.f);
	m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(0.06f, m_data.winghalflength() * 2.f);
	m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(0.06f, m_data.winghalflength() * 2.f);
	
	{ // HOIST_POINTS
		btTransform trA;
		trA.setIdentity();
		trA.setOrigin(btVector3(0.0f,00.f,0.0f));
		trA *= m_local_info.hoistTransform; 
		m_hoist_bodies[HOIST_POINT_0] = localCreateRigidBody(
			0.f,
			trA,
			m_hoist_shapes[HOIST_POINT_0]);

		btScalar xy_angle = btRadians(m_data.hoistanglexy());
		btScalar zxy_angle = btRadians(m_data.hoistanglezxy());

		trA.setIdentity();
		trA.setOrigin(
			btVector3(
			btCos(xy_angle) * btSin(zxy_angle) * m_data.pelvishalflength() + 0.2f*btCos(zxy_angle), 
			btSin(xy_angle) * btSin(zxy_angle) * m_data.pelvishalflength(),
			btCos(zxy_angle) * m_data.pelvishalflength() - 0.2f*btSin(zxy_angle)
			));
		trA *= m_local_info.hoistTransform;

		m_hoist_bodies[HOIST_POINT_1] = localCreateRigidBody(
			0,
			trA,
			m_hoist_shapes[HOIST_POINT_1]);

		trA.setIdentity();
		trA.setOrigin(
			btVector3(
			btCos(xy_angle) * btSin(zxy_angle) * m_data.pelvishalflength() - 0.2f*btCos(zxy_angle), 
			btSin(xy_angle) * btSin(zxy_angle) * m_data.pelvishalflength(),
			btCos(zxy_angle) * m_data.pelvishalflength() + 0.2f*btSin(zxy_angle)
			));
		trA *= m_local_info.hoistTransform;
		m_hoist_bodies[HOIST_POINT_2] = localCreateRigidBody(
			0,
			trA,
			m_hoist_shapes[HOIST_POINT_2]);
	}

	{ // PELVIS
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(
			m_data.pelvismass(),
			m_local_info.startTransform,
			m_shapes[BODYPART_PELVIS]);
	}

	
	{ // UPPER_ARMS
		btTransform transform;
		btScalar xpos = m_data.winghalflength();
		transform.setIdentity();
		transform.setOrigin(btVector3(0.f - xpos, 0.f, 0.f));
		transform.getBasis().setEulerZYX(0, 0, btRadians(-90.f));
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(
			m_data.wingmass(),
			m_local_info.startTransform * transform,
			m_shapes[BODYPART_LEFT_UPPER_ARM]);
	
		transform.setIdentity();
		transform.setOrigin(btVector3(0.f + xpos, 0.f, 0.f));
		transform.getBasis().setEulerZYX(0, 0, btRadians(+90.f));
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(
			m_data.wingmass(),
			m_local_info.startTransform * transform,
			m_shapes[BODYPART_RIGHT_UPPER_ARM]);
	}

	for (int i = 0; i < BODYPART_COUNT; ++i) {
		m_bodies[i]->setDamping(0.00, 0.80);  // setup angular damping.

		// Disable sleeping bodies!
		m_bodies[i]->setDeactivationTime(900.0);
		m_bodies[i]->setSleepingThresholds(0.0, 0.0); 
	}

	// Make sure pelvis is damped, so that wing motor does
	// not cause the pelvis to rotate back and forth!
	m_bodies[BODYPART_PELVIS]->setDamping(0.00, 0.90);
	
	// Setup hoist constraint.
	btPoint2PointConstraint* pointC;

	{ // Hoist constraint to body.
		pointC = new btPoint2PointConstraint(
			*m_bodies[BODYPART_PELVIS],
			*m_hoist_bodies[HOIST_POINT_0],
			btVector3(0.0f,0.0f,0.0f),
			btVector3(0.0f,5.0f,0.0f)
			);
		m_ownerWorld->addConstraint(pointC,true);
		m_hoist_joints[JOINT_HOIST_POINT_0] = pointC;

		pointC = new btPoint2PointConstraint(
			*m_bodies[BODYPART_PELVIS],
			*m_hoist_bodies[HOIST_POINT_1],
			btVector3(+0.2,m_data.pelvishalflength(),0.0f),
			btVector3(
			0,
			5.0f,
			0
			));
		m_ownerWorld->addConstraint(pointC,true);
		m_hoist_joints[JOINT_HOIST_POINT_1] = pointC;

		pointC = new btPoint2PointConstraint(
			*m_bodies[BODYPART_PELVIS],
			*m_hoist_bodies[HOIST_POINT_2],
			btVector3(-0.2f,m_data.pelvishalflength(),0.0f),
			btVector3(
			0,
			5.0f,
			0
			));
		m_ownerWorld->addConstraint(pointC,true);
		m_hoist_joints[JOINT_HOIST_POINT_2] = pointC;
	}

	// Now setup the constraints
	btHingeConstraint* hingeC;

	{ // LEFT_SHOULDER
		hingeC = new btHingeConstraint(
				*m_bodies[BODYPART_LEFT_UPPER_ARM],
				*m_bodies[BODYPART_PELVIS],
				btVector3(0.f, 0.f - m_data.winghalflength(), 0.f),
				make_btVector3(m_data.pelvisrelpostoattachwing()),
				btVector3( 0.f,  0.f, -1.f),
				btVector3( 0.f, +1.f,  0.f)
				);
		hingeC->setLimit(btRadians(90.f - m_data.wingflaphingelimit()), btRadians(90.f + m_data.wingflaphingelimit()));
		hingeC->enableMotor(kEnableMotors);
		hingeC->enableFeedback(kEnableMotors);
		hingeC->setMaxMotorImpulse(m_data.wingflapmotormaximpulse());
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER] = hingeC;
	}
	
	{ // RIGHT_SHOULDER
		hingeC = new btHingeConstraint(
				*m_bodies[BODYPART_RIGHT_UPPER_ARM],	
				*m_bodies[BODYPART_PELVIS],	
				btVector3(0.f, 0.f - m_data.winghalflength(), 0.f),
				make_btVector3(m_data.pelvisrelpostoattachwing()),
				btVector3( 0.f,  0.f, +1.f),
				btVector3( 0.f, +1.f,  0.f)
				);
		hingeC->setLimit(btRadians(90.f - m_data.wingflaphingelimit()), btRadians(90.f + m_data.wingflaphingelimit()));
		hingeC->enableMotor(kEnableMotors);
		hingeC->enableFeedback(kEnableMotors);
		hingeC->setMaxMotorImpulse(m_data.wingflapmotormaximpulse());
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER] = hingeC;
	}

	{  // LEFT FEATHER
		btRigidBody* rb = m_bodies[BODYPART_LEFT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_LEFT_UPPER_ARM_1] = bigfeather;

		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, 0.0f, 0),
				make_btVector3(m_data.featherrelpostoattachfeather()),
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(kEnableMotors);
		hingeC->enableFeedback(kEnableMotors);
		hingeC->setMaxMotorImpulse(m_data.featheraoamotormaximpulse());
		hingeC->setLimit(btRadians(90.f - m_data.featheraoahingelimit()), btRadians(90 + m_data.featheraoahingelimit()));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER_FEATHER_1] = hingeC;
	}

	{  // RIGHT FEATHER
		btRigidBody* rb = m_bodies[BODYPART_RIGHT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_RIGHT_UPPER_ARM_1] = bigfeather;
		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, 0.0f, 0),
				make_btVector3(m_data.featherrelpostoattachfeather()),
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(kEnableMotors);
		hingeC->enableFeedback(kEnableMotors);
		hingeC->setMaxMotorImpulse(m_data.featheraoamotormaximpulse());
		hingeC->setLimit(btRadians(90.f - m_data.featheraoahingelimit()), btRadians(90 + m_data.featheraoahingelimit()));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER_FEATHER_1] = hingeC;
	}
}

BigBird::~BigBird() {
	int i;

	for ( i = 0; i < JOINT_HOIST_COUNT; ++i) {
		m_ownerWorld->removeConstraint(m_hoist_joints[i]);
		delete m_hoist_joints[i];
		m_hoist_joints[i] = 0;
	}

	// Remove all constraints
	for ( i = 0; i < JOINT_COUNT; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i];
		m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for ( i = 0; i < HOIST_POINT_COUNT; ++i) {
		m_ownerWorld->removeRigidBody(m_hoist_bodies[i]);
		delete m_hoist_bodies[i]->getMotionState();
		delete m_hoist_bodies[i];
		m_hoist_bodies[i] = 0;
		delete m_hoist_shapes[i];
		m_hoist_shapes[i] = 0;
	}

	// Remove all bodies and shapes
	for ( i = 0; i < BODYPART_COUNT; ++i) {
		m_ownerWorld->removeRigidBody(m_bodies[i]);
		delete m_bodies[i]->getMotionState();
		delete m_bodies[i];
		m_bodies[i] = 0;
		delete m_shapes[i];
		m_shapes[i] = 0;
	}

	for (int ii = 0; ii < FEATHER_COUNT; ++ii) {
		delete m_feathers[ii];
		m_feathers[ii] = 0;
	}
}

void BigBird::pretick (btScalar dt) {
	int wingbeat_pattern_length = m_data.wingbeatdata().sample_size();
	// Keep track of time.
	m_time += dt;
	m_time_steps = (m_time_steps  + 1) % wingbeat_pattern_length;
	
	if ((2 < m_time) && (m_time <= (2 + dt))) {
		for (int ii = 0; ii < JOINT_HOIST_COUNT; ++ii) {
			m_ownerWorld->removeConstraint(m_hoist_joints[ii]);
		}
	}

	/*if (m_time > (5 + dt)) {
		pretickOutputToFile();
	}*/

	// Compute aerodynamic forces for each feather.
	for (int ii = 0; ii < FEATHER_COUNT; ++ii) {
		m_feathers[ii]->pretick(dt);
	}

	{ // Wing flapping
		//btScalar req_angle = m_info.wingFlapHingeLimit * btSin(m_time * SIMD_2_PI * m_info.wingFlapFrequency);
		btScalar req_angle = m_data.wingbeatdata().sample(m_time_steps % wingbeat_pattern_length).wing();
		((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(90  - req_angle), dt);
		((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER] )->setMotorTarget(btRadians(90  + req_angle), dt);
	}

	{ // Feather angle of attack
		//btScalar feather_angle = m_info.featherAoAHingeLimit * btCos(m_time * SIMD_2_PI * m_info.wingFlapFrequency);
		btScalar feather_angle = m_data.wingbeatdata().sample(m_time_steps % wingbeat_pattern_length).feather();
		((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER_FEATHER_1 ])->setMotorTarget(btRadians(90 + feather_angle), dt);
		((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER_FEATHER_1])->setMotorTarget(btRadians(90 - feather_angle), dt);
	}
	
	/*{ // Feather angle of attack
		//btScalar feather_angle = m_info.featherAoAHingeLimit/5.0f * btCos(m_time * SIMD_2_PI * m_info.wingFlapFrequency);
		btScalar feather_angle = m_info.reqFeatherAngleOfAttack2[m_time_steps % m_info.numPoints];
		((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER_FEATHER_2 ])->setMotorTarget(btRadians(90 + feather_angle), dt);
		((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER_FEATHER_2])->setMotorTarget(btRadians(90 - feather_angle), dt);
	}

	{ // Feather angle of attack
		//btScalar feather_angle = m_info.featherAoAHingeLimit * btCos(m_time * SIMD_2_PI * m_info.wingFlapFrequency);
		btScalar feather_angle = m_info.reqFeatherAngleOfAttack3[m_time_steps % m_info.numPoints];
		((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER_FEATHER_3 ])->setMotorTarget(btRadians(90 + feather_angle), dt);
		((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER_FEATHER_3])->setMotorTarget(btRadians(90 - feather_angle), dt);
	}*/
}

void BigBird::getCurrentTrajectory(proto::TrajectorySample* trajectory_sample) {
	btVector3 posInWorld = m_bodies[BODYPART_PELVIS]->getWorldTransform().getOrigin();
	btScalar lfShoulderImpulse = m_joints[JOINT_LEFT_SHOULDER]->getAppliedImpulse();
	btScalar rtShoulderImpulse = m_joints[JOINT_RIGHT_SHOULDER]->getAppliedImpulse();
	btScalar lfFeatherImpulse = m_joints[JOINT_LEFT_SHOULDER_FEATHER_1]->getAppliedImpulse();
	btScalar rtFeatherImpulse = m_joints[JOINT_RIGHT_SHOULDER_FEATHER_1]->getAppliedImpulse();

	make_Vector3d(posInWorld, trajectory_sample->mutable_pelvisposition());
	trajectory_sample->set_leftwingimpulse(lfShoulderImpulse);
	trajectory_sample->set_rightwingimpulse(rtShoulderImpulse);
	trajectory_sample->set_leftfeatherimpulse(lfFeatherImpulse);
	trajectory_sample->set_rightfeatherimpulse(rtFeatherImpulse);
}