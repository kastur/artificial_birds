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
<<<<<<< Updated upstream
	BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btRigidBody* m_limb,btScalar featherZHalf=0.2,btScalar featherYHalf=0.01,btScalar featherXHalf=0.45);
=======
	BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btRigidBody* m_limb, int id, btScalar x=.45, btScalar y=.01,btScalar z=.2);
>>>>>>> Stashed changes
	virtual ~BigFeather();
	void pretick(btScalar dt);
	void applyImpulse();
	void setScaler(btScalar value);
	void setWindVelocity(const btVector3& value);
	void orient(btScalar angle);
	btRigidBody* getFeatherBody();

	bool aero_on;

protected:
	btRigidBody* BigFeather::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
	btVector3 getEffectiveAirVelocity();
private:
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];

	btScalar t;
	btScalar m_scaler;
	btVector3 m_wind_velocity;
	int m_index;
	btRigidBody* m_limb;
	int m_id;
};


#endif
