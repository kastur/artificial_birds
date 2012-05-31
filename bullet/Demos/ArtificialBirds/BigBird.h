#pragma once
#ifndef BIGBIRD_H__
#define BIGBIRD_H__

#include "btBulletDynamicsCommon.h"
#include "BigFeather.h"

class BigBird {
	enum
	{
		BODYPART_PELVIS = 0,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,
		BODYPART_LEFT_WRIST,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,
		BODYPART_RIGHT_WRIST,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_SHOULDER_FEATHER,
		JOINT_LEFT_ELBOW,
		JOINT_LEFT_WRIST,
		

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_SHOULDER_FEATHER,
		JOINT_RIGHT_ELBOW,
		JOINT_RIGHT_WRIST,
		

		JOINT_COUNT
	};

public:
	BigBird (btDynamicsWorld* ownerWorld, const btVector3& positionOffset);
	virtual ~BigBird();
	void pretick(btScalar dt);
	void applyImpulse();
	void applyFeatherImpulse();
	const btVector3& getPosition() { return m_bodies[BODYPART_PELVIS]->getWorldTransform().getOrigin();  }


	void TrimUp() {
		m_feather_angle += 0.5f;
	}

	void TrimDn() {
		m_feather_angle -= 0.5f;
	}

	void WindUp() {
		for (int ii = 0; ii < m_feathers.size(); ++ii) {
			m_feathers[ii]->WindUp();
		}
	}

	void WindDn() {
		for (int ii = 0; ii < m_feathers.size(); ++ii) {
			m_feathers[ii]->WindDn();
		}
	}

protected:
	btRigidBody* BigBird::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
	void addFeather(btRigidBody* rb, const btVector3& relPos, btScalar rbAngleX, btScalar rbAngleY, btScalar featherAngle, btScalar featherGive);
	

private:
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
	btAlignedObjectArray<class BigFeather*> m_feathers;
	btAlignedObjectArray<class btTypedConstraint*> m_featherjoints;

	btScalar t;  // keep track of time.

	btScalar m_feather_angle;

};

#endif
