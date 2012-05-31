#include <iostream>
#include <iomanip>

#include "BigFeather.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f

btRigidBody* BigFeather::localCreateRigidBody(
	btScalar mass,
	const btTransform& startTransform,
	btCollisionShape* shape) {
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

BigFeather::BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btRigidBody* limb, int id, btScalar x, btScalar y, btScalar z)
	:m_ownerWorld (ownerWorld), m_limb(limb), m_id(id) {
		t = 0;
		m_scaler = 8.0f;
		m_wind_velocity = btVector3(-3.0, 0, 0);

		// Calculate offset transform.
		btTransform offset;
		offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;  // used for setting rigid body positions.

		// Create spine.
		transform.setIdentity();
		transform.setOrigin(btVector3(0,1,0));
		m_shapes[BODYPART_SPINE] = new btBoxShape(btVector3(x, y, z));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(0.1, offset*transform, m_shapes[BODYPART_SPINE]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.09, 0.09);
			m_bodies[i]->setDeactivationTime(900.0);
			m_bodies[i]->setSleepingThresholds(0, 0);
		}
		
	}

BigFeather::~BigFeather() {

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

btVector3 BigFeather::getEffectiveAirVelocity() {
	btRigidBody* body = this->m_bodies[BODYPART_SPINE];
	btVector3 world_vel = body->getVelocityInLocalPoint(btVector3(0,0,0));
	return body->getWorldTransform().getBasis().transpose() * (m_wind_velocity - world_vel);
}

void BigFeather::pretick(btScalar dt) {
	t += dt; // keep track of time.

	btRigidBody* feather = this->m_bodies[BODYPART_SPINE];
	feather->activate();

	const btVector3 air_velocity = getEffectiveAirVelocity();
	btVector3 surface_normal(0,1,0);

	// Decompose air velocity into components tangential and normal to surface.
	btScalar angle_vn = btAngle(air_velocity, surface_normal);

	if (angle_vn > SIMD_PI/2.0f) {
		surface_normal = -surface_normal;
		angle_vn = btAngle(air_velocity, surface_normal);
	}
	 
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
	/*
	btVector3 impulse = (lift_impulse + drag_impulse);
	std::cout << std::setprecision(2) << std::fixed;
	std::cout << "  force_mag: " << impulse.length();
	std::cout << "  x=" << impulse.x();
	std::cout << "  y=" << impulse.y();
	std::cout << "  z=" << impulse.z();
	std::cout << std::endl;
	*/

	btVector3 liftForce = (lift_impulse + drag_impulse) / m_scaler;
	btVector3 forcePos =  m_limb->getCenterOfMassPosition() - feather->getCenterOfMassPosition();
	
	const btScalar maxForce = 1000.0;
	btScalar forceMag = liftForce.length();
	if (forceMag < maxForce) {
		m_limb->applyForce(liftForce, btVector3(0,0,0));
		std::cout << std::setprecision(2) << std::fixed;
		std::cout << m_id << "  AoA: " << angle_of_attack * 180 / SIMD_PI << " l: " << liftForce.x() << " " << liftForce.y() << " " << liftForce.z() << std::endl;
	} else {
		std::cout << "sat!";
	}

	
}

void BigFeather::applyImpulse() {
	btRigidBody* feather = this->m_bodies[BODYPART_SPINE];
	m_limb->activate();

	btVector3 forcePos =  m_limb->getCenterOfMassPosition() - feather->getCenterOfMassPosition();

	if (m_limb)
		m_limb->applyForce(btVector3(0, 50, 0), forcePos);

	std::cout << "Applied impulse!" << std::endl;
}

void BigFeather::orient(btScalar angle) {
	btRigidBody* feather = this->m_bodies[BODYPART_SPINE];
	feather->activate();
	btTransform trs;
	trs.setIdentity();
	trs.setOrigin(btVector3(0,2,0));
	trs.setRotation(btQuaternion(btVector3(0,0,1), angle));
	
	feather->setLinearVelocity(btVector3(0,0,0));
	feather->setAngularVelocity(btVector3(0,0,0));
	feather->setWorldTransform(trs);
	t = 0;
}

btRigidBody* BigFeather::getFeatherBody() {
	return this->m_bodies[BODYPART_SPINE];
}

void BigFeather::setScaler(btScalar value)
{
	m_scaler = value;
}

void BigFeather::setWindVelocity(const btVector3& value)
{
	m_wind_velocity = value;
}
