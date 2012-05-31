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
		JOINT_LEFT_ELBOW,
		JOINT_LEFT_WRIST,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,
		JOINT_RIGHT_WRIST,

		JOINT_COUNT
	};

public:
	BigBird (btDynamicsWorld* ownerWorld, const btVector3& positionOffset);
	virtual ~BigBird();
	void pretick(btScalar dt);
	void applyForImpulse();
	void applyUpImpulse();
	void applyDownImpulse();
	void zeroImpulse();
	void toggleMotors();
	void upTailAngle();
	void downTailAngle();
	void applyFeatherImpulse();
	const btVector3& getPosition() { return m_bodies[BODYPART_PELVIS]->getWorldTransform().getOrigin();  }

protected:
	btRigidBody* BigBird::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
	void addFeather(btRigidBody* rb, const btVector3& relPos, btScalar rbAngleX, btScalar rbAngleY, btScalar featherAngle, btScalar featherGive, int id);
	void addTailFeather(btRigidBody* rb, const btVector3& relPos, btScalar rbAngleX, btScalar rbAngleY, btScalar featherAngle, btScalar featherGive, int id);
	

private:

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
	btAlignedObjectArray<class BigFeather*> m_feathers;
	btAlignedObjectArray<class btTypedConstraint*> m_featherjoints;

	bool motor_state;
	btVector3 impulse_pretick;
	btVector3 impulse_relpos;
	btScalar t;  // keep track of time.

	btHingeConstraint* tailFeather;
	btScalar tailAngle;
};

#endif
