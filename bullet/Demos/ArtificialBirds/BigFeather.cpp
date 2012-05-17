#include <iostream>
#include <iomanip>

#include "BigFeather.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f

btRigidBody* BigFeather::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
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

BigFeather::BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, int index) : m_ownerWorld (ownerWorld) {
		t = 0;
		m_index = index;

		// Calculate offset transform.
		btTransform offset;
		offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;  // used for setting rigid body positions.

		// Create head.
		//transform.setIdentity();
		//transform.setOrigin(btVector3(0,3,0));
		//m_shapes[BODYPART_HEAD] = new btSphereShape(0.10);
		//m_bodies[BODYPART_HEAD] = localCreateRigidBody(0, offset*transform, m_shapes[BODYPART_HEAD]);
		m_shapes[BODYPART_HEAD] = 0;
		m_bodies[BODYPART_HEAD] = 0;

		// Create pelvis.
		//transform.setIdentity();
		//transform.setOrigin(btVector3(0,2,0));
		//m_shapes[BODYPART_PELVIS] = new btBoxShape(btVector3(0.40, 0.01, 0.20));
		//m_bodies[BODYPART_PELVIS] = localCreateRigidBody(1, offset*transform, m_shapes[BODYPART_PELVIS]);
		m_shapes[BODYPART_PELVIS] = 0;
		m_bodies[BODYPART_PELVIS] = 0;

		// Create spine.
		transform.setIdentity();
		transform.setOrigin(btVector3(0,1,0));
		m_shapes[BODYPART_SPINE] = new btBoxShape(btVector3(0.40, 0.01, 0.07));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(0.1, offset*transform, m_shapes[BODYPART_SPINE]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			if (!m_bodies[i]) continue;
			m_bodies[i]->setDamping(0.09, 0.09);
			m_bodies[i]->setDeactivationTime(900.0);
			m_bodies[i]->setSleepingThresholds(0, 0);
		}
		

		/*
		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(+0.40, 0.00, 0.00));
		localB.getBasis().setEulerZYX(0,0,0); localB.setOrigin(btVector3(-0.10, 0.00, 0.00));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(btRadians(90), btRadians(45), btRadians(90));
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(+0.40, 0.00, 0.00));
		localB.getBasis().setEulerZYX(0,0,0); localB.setOrigin(btVector3(-0.40, 0.00, 0.00));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(btRadians(-45)), btScalar(btRadians(90)));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
		*/
	}

BigFeather::~BigFeather() {

	// Remove constraints.
	for ( int ii = 0; ii < JOINT_COUNT; ++ii) {
		if (!m_joints[ii]) continue;
		m_ownerWorld->removeConstraint(m_joints[ii]);
		delete m_joints[ii];
		m_joints[ii] = 0;
	}

	// Remove bodies.
	for (int ii = 0; ii < BODYPART_COUNT; ++ii) {
		if (!m_bodies[ii]) continue;
		m_ownerWorld->removeRigidBody(m_bodies[ii]);
		delete m_bodies[ii]->getMotionState();
		delete m_bodies[ii];
		m_bodies[ii] = 0;
	}

	// Remove collision shapes.
	for (int ii = 0; ii < BODYPART_COUNT; ++ii) {
		if (!m_shapes[ii]) continue;
		delete m_shapes[ii];
		m_shapes[ii] = 0;
	}
}


btVector3 getVelocityInLocalFrame(btRigidBody* body,const btVector3& relpos) {
	const btVector3 wvel=body->getVelocityInLocalPoint(relpos);
	return body->getWorldTransform().getBasis().transpose() * wvel;
}

void BigFeather::pretick(btScalar dt) {
	t += dt; // keep track of time.

	btRigidBody* feather = this->m_bodies[BODYPART_SPINE];

	const btVector3 air_velocity = -1 * getVelocityInLocalFrame(feather, btVector3(0,0,0));
	const btVector3 surface_normal(0,1,0);

	// Decompose air velocity into components tangential and normal to surface.
	btScalar angle_vn = btAngle(air_velocity, surface_normal);
	btVector3 vn = (air_velocity.length()*btCos(angle_vn))*surface_normal;
	btVector3 vt = air_velocity - vn;

	// Calculate the angle of attack.
	btScalar angle_of_attack = btAtan(btDot(vn,surface_normal)/vt.length());	

	// Calculate lift and drag as a function of the angle of attack.
	btScalar drag_coeff = -btCos(2 * angle_of_attack) + 1.0f;
	btScalar pp = 3.32 * angle_of_attack + 0.112;
	btScalar lift_coeff = 4*((1/(1+btExp(-0.5*pp)) - 0.5)*1.9 + btSin(1.4*pp) * btExp(-abs(2*pp)))*btExp(-abs(0.3*pp)) + 0.1;
	
	// Calculate the directions of drag and lift.
	btVector3 drag_direction = btVector3(air_velocity).normalize();
	btVector3 lift_direction = btCross(btCross(drag_direction, surface_normal).normalize(), drag_direction);

	// Compute the lift and drag forces, and convert to the world transform.
	btVector3 lift_impulse = lift_coeff * air_velocity.length2() * lift_direction;
	btVector3 drag_impulse = drag_coeff * air_velocity.length2() * drag_direction;
	lift_impulse = feather->getWorldTransform().getBasis() * lift_impulse;
	drag_impulse = feather->getWorldTransform().getBasis() * drag_impulse;

	btVector3 world_air_vel = feather->getWorldTransform().getBasis() * air_velocity;
	btVector3 feather_pos = feather->getWorldTransform().getOrigin();

	// DEBUG
	/*
	std::cout << std::setprecision(2) << std::fixed << t;
	std::cout << angle_of_attack * SIMD_DEGS_PER_RAD;
	std::cout << std::setw(7) << air_velocity.y();
	std::cout << std::setw(7) << lift_impulse.length();
	std::cout << std::setw(7) << drag_impulse.length();
	std::cout << "  air: "  << world_air_vel.x() << " " << world_air_vel.y() << " " << world_air_vel.z();
	std::cout << "  dc: " << drag_coeff << " d: " << drag_impulse.x() << " " << drag_impulse.y() << " " << drag_impulse.z();
	std::cout << "  lc: " << lift_coeff << " l: " << lift_impulse.x() << " " << lift_impulse.y() << " " << lift_impulse.z();
	
	std::cout << "  pos: " << feather_pos.x() << " " << feather_pos.y() << " " << feather_pos.z();
	std::cout << "  force_mag: " << (lift_impulse + drag_impulse).length(); 
	std::cout << std::endl;
	*/
	if ((lift_impulse + drag_impulse).length() < 10000) {
		feather->applyForce((lift_impulse + drag_impulse)/5.0, btVector3(0,0,0));
	}
}

void BigFeather::applyImpulse() {
	
	btRigidBody* feather = this->m_bodies[BODYPART_SPINE];
	feather->activate();
	feather->applyImpulse(btVector3(-10,3,0), btVector3(0,0,0));
}

void BigFeather::orient(btScalar angle) {
	btRigidBody* feather = this->m_bodies[BODYPART_SPINE];
	feather->activate();
	btTransform trs;
	trs.setIdentity();
	trs.setOrigin(btVector3(0,2,m_index));
	trs.setRotation(btQuaternion(btVector3(0,0,1), angle));
	
	feather->setLinearVelocity(btVector3(0,0,0));
	feather->setAngularVelocity(btVector3(0,0,0));
	feather->setWorldTransform(trs);
	t = 0;
}

btRigidBody* BigFeather::getFeatherBody() {
	return this->m_bodies[BODYPART_SPINE];
}