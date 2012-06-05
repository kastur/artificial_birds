#ifndef PROTO_HELPER_H__
#define PROTO_HELPER_H__

#include "btBulletDynamicsCommon.h"
#include "proto.pb.h"

btVector3 make_btVector3(const proto::Vector3d& proto_vector);
void make_Vector3d(const btVector3& vector, proto::Vector3d* proto_vector);

#endif