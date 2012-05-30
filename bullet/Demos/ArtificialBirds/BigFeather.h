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
	BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btRigidBody* m_limb,btScalar featherZHalf=0.2,btScalar featherYHalf=0.01,btScalar featherXHalf=0.45);
	virtual ~BigFeather();
	void pretick(btScalar dt);
	void applyImpulse();
	void orient(btScalar angle);
	btRigidBody* getFeatherBody();

	bool aero_on;

protected:
	btRigidBody* BigFeather::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

private:
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];

	btScalar t;
	int m_index;
	btRigidBody* m_limb;
};


#endif
