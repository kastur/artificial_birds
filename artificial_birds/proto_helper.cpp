#include "proto_helper.h"
btVector3 make_btVector3(const proto::Vector3d& proto_vector) {
	return btVector3(proto_vector.x(), proto_vector.y(), proto_vector.z());
}

void make_Vector3d(const btVector3& vector, proto::Vector3d* proto_vector) {
	proto_vector->set_x(vector.x());
	proto_vector->set_y(vector.y());
	proto_vector->set_z(vector.z());
}