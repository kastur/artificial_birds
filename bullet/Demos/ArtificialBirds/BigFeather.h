#pragma once
#ifndef BIGFEATHER_H__
#define BIGFEATHER_H__

#include "btBulletDynamicsCommon.h"

class BigFeather {
	enum {
		BODYPART_SPINE = 0,
		BODYPART_COUNT
	};

	enum {
		JOINT_COUNT = 0
	};

public:
	BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btRigidBody* m_limb);
	virtual ~BigFeather();
	void pretick(btScalar dt);
	void applyImpulse();
	void orient(btScalar angle);
	btRigidBody* getFeatherBody();

	void WindUp() {
		btScalar wind = m_wind_vel.x();
		wind += 1.0;
		m_wind_vel.setX(wind);
	}

	void WindDn() {
		btScalar wind = m_wind_vel.x();
		wind -= 1.0;
		m_wind_vel.setX(wind);
	}



protected:
	btRigidBody* BigFeather::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

private:
	btVector3 getEffectiveWindVelocity();

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];

	btScalar t;
	int m_index;
	btRigidBody* m_limb;
	btVector3 m_wind_vel;
};


#endif
