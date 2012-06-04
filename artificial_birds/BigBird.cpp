#include "BigBird.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f

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

BigBird::BigBird(btDynamicsWorld* ownerWorld, const BigBirdConstructionInfo& bbInfo) :
	m_ownerWorld(ownerWorld),
	m_info(bbInfo) {
		start();
	}

void BigBird::restart() {
	end();
	m_info.birdId++;
	start();
}

void BigBird::start() {
	bool enable_motor = true;
	m_time = 0;
	m_time_steps = -1;

	m_hoist_shapes[HOIST_POINT_0] = new btCapsuleShape(0.101f,0.101f);
	m_hoist_shapes[HOIST_POINT_1] = new btCapsuleShape(0.101f,0.101f);
	m_hoist_shapes[HOIST_POINT_2] = new btCapsuleShape(0.101f,0.101f);

	m_shapes[BODYPART_PELVIS] = new btCapsuleShape(0.1f, m_info.pelvisHalfLength*2.f);
	m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(0.06f, m_info.wingHalfLength*2.f);
	m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(0.06f, m_info.wingHalfLength*2.f);
	
	{ // HOIST point 0
		btTransform trA;
		btTransform trB;
		btTransform trC;
		trA.setIdentity();
		trA.setOrigin(btVector3(0.0f,00.f,0.0f));
		trA *= m_info.hoistTransform; 
		m_hoist_bodies[HOIST_POINT_0] = localCreateRigidBody(
			0.f,
			trA,
			m_hoist_shapes[HOIST_POINT_0]);

		trA.setIdentity();
		trA.setOrigin(
			btVector3(
			btCos(btRadians(m_info.hoistAngleXY))*btSin(btRadians(m_info.hoistAngleZXY))* m_info.pelvisHalfLength + 0.2f*btCos(btRadians(m_info.hoistAngleZXY)), 
			btSin(btRadians(m_info.hoistAngleXY))*btSin(btRadians(m_info.hoistAngleZXY))* m_info.pelvisHalfLength,
			btCos(btRadians(m_info.hoistAngleZXY))*m_info.pelvisHalfLength - 0.2f*btSin(btRadians(m_info.hoistAngleZXY))
			));
		trA *= m_info.hoistTransform;

		m_hoist_bodies[HOIST_POINT_1] = localCreateRigidBody(
			0,
			trA,
			m_hoist_shapes[HOIST_POINT_1]);

		trA.setIdentity();
		trA.setOrigin(
			btVector3(
			btCos(btRadians(m_info.hoistAngleXY))*btSin(btRadians(m_info.hoistAngleZXY))* m_info.pelvisHalfLength - 0.2f*btCos(btRadians(m_info.hoistAngleZXY)), 
			btSin(btRadians(m_info.hoistAngleXY))*btSin(btRadians(m_info.hoistAngleZXY))* m_info.pelvisHalfLength,
			btCos(btRadians(m_info.hoistAngleZXY))*m_info.pelvisHalfLength + 0.2f*btSin(btRadians(m_info.hoistAngleZXY))
			));
		trA *= m_info.hoistTransform;
		m_hoist_bodies[HOIST_POINT_2] = localCreateRigidBody(
			0,
			trA,
			m_hoist_shapes[HOIST_POINT_2]);
	}/**/

	{ // PELVIS
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(
			m_info.pelvisMass,
			m_info.startTransform,
			m_shapes[BODYPART_PELVIS]);
	}

	
	{ // UPPER_ARMS
		btTransform transform;
		btScalar xpos = m_info.wingHalfLength;
		transform.setIdentity();
		transform.setOrigin(btVector3(0.f - xpos, 0.f, 0.f));
		transform.getBasis().setEulerZYX(0, 0, btRadians(-90.f));
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(
			m_info.wingMass,
			m_info.startTransform * transform,
			m_shapes[BODYPART_LEFT_UPPER_ARM]);
	
		transform.setIdentity();
		transform.setOrigin(btVector3(0.f + xpos, 0.f, 0.f));
		transform.getBasis().setEulerZYX(0, 0, btRadians(+90.f));
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(
			m_info.wingMass,
			m_info.startTransform * transform,
			m_shapes[BODYPART_RIGHT_UPPER_ARM]);
	}

	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.00, 0.80);  // setup angular damping.

		// Disable sleeping bodies!
		m_bodies[i]->setDeactivationTime(900.0);
		m_bodies[i]->setSleepingThresholds(0.0, 0.0); 
	}

	// Make sure pelvis is damped, so that wing motor does
	// not cause the pelvis to rotate back and forth!
	m_bodies[BODYPART_PELVIS]->setDamping(0.00, 0.90);
	
	//setup hoist constraint
	btPoint2PointConstraint* pointC;

	{ //Hoist constraint to body
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
			btVector3(+0.2,1.0f,0.0f),
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
			btVector3(-0.2f,1.0f,0.0f),
			btVector3(
			0,
			5.0f,
			0
			));
		m_ownerWorld->addConstraint(pointC,true);
		m_hoist_joints[JOINT_HOIST_POINT_2] = pointC;
	}/**/

	// Now setup the constraints
	btHingeConstraint* hingeC;

	{ // LEFT_SHOULDER
		hingeC = new btHingeConstraint(
				*m_bodies[BODYPART_LEFT_UPPER_ARM],	
				*m_bodies[BODYPART_PELVIS],	
				btVector3(0.f, 0.f - m_info.wingHalfLength - 0.1f, 0.f),
				m_info.pelvisRelPosToAttachWing,
				btVector3( 0.f,  0.f, -1.f),
				btVector3( 0.f, +1.f,  0.f)
				);
		hingeC->setLimit(btRadians(90.f - m_info.wingFlapHingeLimit), btRadians(90.f + m_info.wingFlapHingeLimit));
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.wingFlapMotorMaxImpulse);
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER] = hingeC;
	}
	
	{ // RIGHT_SHOULDER
		hingeC = new btHingeConstraint(
				*m_bodies[BODYPART_RIGHT_UPPER_ARM],	
				*m_bodies[BODYPART_PELVIS],	
				btVector3(0.f, 0.f - m_info.wingHalfLength - 0.1f, 0.f),
				m_info.pelvisRelPosToAttachWing,
				btVector3( 0.f,  0.f, +1.f),
				btVector3( 0.f, +1.f,  0.f)
				);
		hingeC->setLimit(btRadians(90.f - m_info.wingFlapHingeLimit), btRadians(90.f + m_info.wingFlapHingeLimit));
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.wingFlapMotorMaxImpulse);
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
				btVector3(0, -0.3f, 0),
				m_info.featherRelPosToAttachFeather,
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.featherAoAMotorMaxImpulse);
		hingeC->setLimit(btRadians(90.f - m_info.featherAoAHingeLimit), btRadians(90 + m_info.featherAoAHingeLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER_FEATHER_1] = hingeC;
	}

	/*{  // LEFT_FEATHER_2
		btRigidBody* rb = m_bodies[BODYPART_LEFT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_LEFT_UPPER_ARM_2] = bigfeather;

		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, 0.0f, 0),
				m_info.featherRelPosToAttachFeather,
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.featherAoAMotorMaxImpulse);
		hingeC->setLimit(btRadians(90.f - m_info.featherAoAHingeLimit), btRadians(90 + m_info.featherAoAHingeLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER_FEATHER_2] = hingeC;
	}

	{  // LEFT_FEATHER_2
		btRigidBody* rb = m_bodies[BODYPART_LEFT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_LEFT_UPPER_ARM_3] = bigfeather;

		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, +0.3f, 0),
				m_info.featherRelPosToAttachFeather,
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.featherAoAMotorMaxImpulse);
		hingeC->setLimit(btRadians(90.f - m_info.featherAoAHingeLimit), btRadians(90 + m_info.featherAoAHingeLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_LEFT_SHOULDER_FEATHER_3] = hingeC;
	}*/

	{  // RIGHT FEATHER
		btRigidBody* rb = m_bodies[BODYPART_RIGHT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_RIGHT_UPPER_ARM_1] = bigfeather;

		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, -0.3f, 0),
				m_info.featherRelPosToAttachFeather,
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.featherAoAMotorMaxImpulse);
		hingeC->setLimit(btRadians(90.f - m_info.featherAoAHingeLimit), btRadians(90 + m_info.featherAoAHingeLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER_FEATHER_1] = hingeC;
	}

	/*{  // RIGHT_FEATHER_2
		btRigidBody* rb = m_bodies[BODYPART_RIGHT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_RIGHT_UPPER_ARM_2] = bigfeather;

		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, 0.0f, 0),
				m_info.featherRelPosToAttachFeather,
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.featherAoAMotorMaxImpulse);
		hingeC->setLimit(btRadians(90.f - m_info.featherAoAHingeLimit), btRadians(90 + m_info.featherAoAHingeLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER_FEATHER_2] = hingeC;
	}

	{  // RIGHT_FEATHER_3
		btRigidBody* rb = m_bodies[BODYPART_RIGHT_UPPER_ARM];
		btVector3 feather_pos = rb->getCenterOfMassPosition();
		BigFeather* bigfeather = new BigFeather(m_ownerWorld, feather_pos, rb);
		m_feathers[FEATHER_RIGHT_UPPER_ARM_3] = bigfeather;

		hingeC = new btHingeConstraint(
				*rb,
				*bigfeather->getFeatherBody(),
				btVector3(0, +0.3f, 0),
				m_info.featherRelPosToAttachFeather,
				btVector3(0, 1, 0),
				btVector3(0, 0, 1)
				);
		hingeC->enableMotor(enable_motor);
		hingeC->enableFeedback(enable_motor);
		hingeC->setMaxMotorImpulse(m_info.featherAoAMotorMaxImpulse);
		hingeC->setLimit(btRadians(90.f - m_info.featherAoAHingeLimit), btRadians(90 + m_info.featherAoAHingeLimit));
		m_ownerWorld->addConstraint(hingeC, true);
		m_joints[JOINT_RIGHT_SHOULDER_FEATHER_3] = hingeC;
	}*/

	convert << "bigBird" <<  m_info.birdId << ".txt";
	file.open(convert.str());
	initialOutputToFile();

}

BigBird::~BigBird() {
	end();
}

void BigBird::end() {
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

	file.close();
}

void BigBird::pretick (btScalar dt) {
	// Keep track of time.
	m_time += dt;
	m_time_steps = (m_time_steps  + 1) % m_info.numPoints;
	
	if ((5 < m_time) && (m_time < (5 + 2*dt))) {
		for (int ii = 0; ii < JOINT_HOIST_COUNT; ++ii) {
			m_ownerWorld->removeConstraint(m_hoist_joints[ii]);
		}
	}/**/

	if (m_time_steps > 10) {
		pretickOutputToFile();
	}

	// Compute aerodynamic forces for each feather.
	for (int ii = 0; ii < FEATHER_COUNT; ++ii) {
		m_feathers[ii]->pretick(dt);
	}

	{ // Wing flapping
		btScalar req_angle = m_info.wingFlapHingeLimit * btSin(m_time * SIMD_2_PI * m_info.wingFlapFrequency);
		//btScalar req_angle = m_info.birdCPG.reqWingFlappingAngle.at(m_time_steps % m_info.numPoints);
		((btHingeConstraint*)m_joints[JOINT_RIGHT_SHOULDER])->setMotorTarget(btRadians(90  - req_angle), dt);
		((btHingeConstraint*)m_joints[JOINT_LEFT_SHOULDER] )->setMotorTarget(btRadians(90  + req_angle), dt);
	}

	{ // Feather angle of attack
		//btScalar feather_angle = 0.f;
		btScalar feather_angle = m_info.featherAoAHingeLimit * btCos(m_time * SIMD_2_PI * m_info.wingFlapFrequency);
		//btScalar feather_angle = m_info.birdCPG.reqFeatherAngleOfAttack1.at(m_time_steps % m_info.numPoints);
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

void BigBird::initialOutputToFile() {

	file << "========== Initial Config ==========" << std::endl;

	file << "id: " << m_info.birdId << std::endl;
	file << "pl,wl: " << m_info.pelvisHalfLength << "," << m_info.wingHalfLength << std::endl;
	file << "pm,wm: " << m_info.pelvisMass << "," << m_info.wingMass << std::endl;
	file << "wfl,fal: " << m_info.wingFlapHingeLimit << "," << m_info.featherAoAHingeLimit << std::endl;

	file << "haXY,haZXY: " << m_info.hoistAngleXY << "," << m_info.hoistAngleZXY << std::endl;
	file << "np,rs: " << m_info.numPoints << "," << m_info.randSeed << std::endl;

	file << "wa: ";
	for (int ii = 0; ii < m_info.numPoints-1; ++ii) {
		file << m_info.birdCPG.reqWingFlappingAngle.at(ii) << ",";
	}
	file << m_info.birdCPG.reqWingFlappingAngle.at(m_info.numPoints-1) << std::endl;

	file << "faoa1: ";
	for (int ii = 0; ii < m_info.numPoints-1; ++ii) {
		file << m_info.birdCPG.reqFeatherAngleOfAttack1.at(ii) << ",";
	}
	file << m_info.birdCPG.reqFeatherAngleOfAttack1.at(m_info.numPoints-1) << std::endl;
	
	file << "========== Initial Config End ==========" << std::endl;

}

void BigBird::pretickOutputToFile() {
	btVector3 posInWorld = m_bodies[BODYPART_PELVIS]->getWorldTransform().getOrigin();
	btQuaternion ortInWorld = m_bodies[BODYPART_PELVIS]->getWorldTransform().getRotation();
	file << posInWorld.getX() << "," << posInWorld.getY() << "," << posInWorld.getZ() << "," << btDegrees(ortInWorld.getAngle()) << std::endl;
	btScalar lfShoulderImpulse = m_joints[JOINT_LEFT_SHOULDER]->getAppliedImpulse();
	btScalar rtShoulderImpulse = m_joints[JOINT_RIGHT_SHOULDER]->getAppliedImpulse();
	btScalar lfFeatherImpulse = m_joints[JOINT_LEFT_SHOULDER_FEATHER_1]->getAppliedImpulse();
	btScalar rtFeatherImpulse = m_joints[JOINT_RIGHT_SHOULDER_FEATHER_1]->getAppliedImpulse();
	file << lfShoulderImpulse << "," << rtShoulderImpulse << "," << lfFeatherImpulse << "," << rtFeatherImpulse << std::endl;
}

void BigBird::fillMetricDetails(MetricDetails* md) {
	if (md) {
		btVector3 posInWorld = m_bodies[BODYPART_PELVIS]->getWorldTransform().getOrigin();
		btScalar lfShoulderImpulse = m_joints[JOINT_LEFT_SHOULDER]->getAppliedImpulse();
		btScalar rtShoulderImpulse = m_joints[JOINT_RIGHT_SHOULDER]->getAppliedImpulse();
		btScalar lfFeatherImpulse = m_joints[JOINT_LEFT_SHOULDER_FEATHER_1]->getAppliedImpulse();
		btScalar rtFeatherImpulse = m_joints[JOINT_RIGHT_SHOULDER_FEATHER_1]->getAppliedImpulse();
		md->pelvisPosition.push_back(posInWorld);
		md->leftWingTorque.push_back(lfShoulderImpulse);
		md->rightWingTorque.push_back(rtShoulderImpulse);
		md->leftFeatherTorque.push_back(lfFeatherImpulse);
		md->rightFeatherTorque.push_back(rtFeatherImpulse);
	}
}