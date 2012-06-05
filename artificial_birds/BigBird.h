#pragma once
#ifndef BIGBIRD_H__
#define BIGBIRD_H__

#include "btBulletDynamicsCommon.h"
#include "BigBirdLocalParams.h"
#include "BigFeather.h"
#include "../proto/proto.pb.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "MetricDetails.h"

class BigBird {

	enum {
		BODYPART_LEFT_UPPER_ARM = 0,
		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_PELVIS,
		BODYPART_COUNT
	};

	enum {
		FEATHER_LEFT_UPPER_ARM_1 = 0,
		FEATHER_RIGHT_UPPER_ARM_1,
		FEATHER_COUNT
	};

	enum {
		JOINT_LEFT_SHOULDER = 0,
		JOINT_RIGHT_SHOULDER,
		JOINT_LEFT_SHOULDER_FEATHER_1,
		JOINT_RIGHT_SHOULDER_FEATHER_1,
		JOINT_COUNT
	};

	enum {
		HOIST_POINT_0 = 0,
		HOIST_POINT_1,
		HOIST_POINT_2,
		HOIST_POINT_COUNT
	};

	enum {
		JOINT_HOIST_POINT_0 = 0,
		JOINT_HOIST_POINT_1,
		JOINT_HOIST_POINT_2,
		JOINT_HOIST_COUNT
	};

public:
	BigBird(
		btDynamicsWorld* ownerWorld,
		const BigBirdLocalParams& local_info,
		const proto::BigBirdConstructionData& data);
	
	virtual ~BigBird();

	void pretick(btScalar dt);

	btVector3 getPosition() {
		return m_bodies[BODYPART_PELVIS]->getCenterOfMassPosition();
	}

	
	btScalar getTime() {return m_time;}
	void getCurrentTrajectory(proto::TrajectorySample* sample);

protected:
	btRigidBody* BigBird::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

private:
	btDynamicsWorld* m_ownerWorld;
	const BigBirdLocalParams& m_local_info;
	const proto::BigBirdConstructionData& m_data;
	
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	BigFeather* m_feathers[FEATHER_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
	btCollisionShape* m_hoist_shapes[HOIST_POINT_COUNT];
	btRigidBody* m_hoist_bodies[HOIST_POINT_COUNT];
	btTypedConstraint* m_hoist_joints[JOINT_HOIST_COUNT];

	btScalar m_time;  // keep track of time.
	int m_time_steps;
};

#endif
