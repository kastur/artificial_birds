#pragma once
#ifndef BIGFEATHER_H__
#define BIGFEATHER_H__

#include "btBulletDynamicsCommon.h"

class BigFeather {
	enum {
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,
		BODYPART_COUNT
	};

	enum {
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,
		JOINT_COUNT
	};

public:
	BigFeather (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btRigidBody* m_limb);
	virtual ~BigFeather();
	void pretick(btScalar dt);
	void applyImpulse();
	void orient(btScalar angle);
	btRigidBody* getFeatherBody();

protected:
	btRigidBody* BigFeather::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

private:
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	btScalar t;
	int m_index;
	btRigidBody* m_limb;
};


#endif
