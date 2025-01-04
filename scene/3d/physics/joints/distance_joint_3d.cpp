#include "distance_joint_3d.h"

void DistanceJoint3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_param", "param", "value"), &DistanceJoint3D::set_param);
	ClassDB::bind_method(D_METHOD("get_param", "param"), &DistanceJoint3D::get_param);

	ADD_PROPERTYI(PropertyInfo(Variant::FLOAT, "distance_limit/upper", PROPERTY_HINT_RANGE, "0.00,10.0,0.1,or_greater,or_less"), "set_param", "get_param", PARAM_LIMIT_UPPER);
	ADD_PROPERTYI(PropertyInfo(Variant::FLOAT, "distance_limit/lower", PROPERTY_HINT_RANGE, "0.00,10.0,0.1,or_greater,or_less"), "set_param", "get_param", PARAM_LIMIT_LOWER);

	BIND_ENUM_CONSTANT(PARAM_LIMIT_UPPER);
	BIND_ENUM_CONSTANT(PARAM_LIMIT_LOWER);
}

void DistanceJoint3D::set_param(Param p_param, real_t p_value) {
	ERR_FAIL_INDEX(p_param, PARAM_MAX);
	params[p_param] = p_value;
	if (is_configured()) {
		PhysicsServer3D::get_singleton()->distance_joint_set_param(get_rid(), PhysicsServer3D::DistanceJointParam(p_param), p_value);
	}

	update_gizmos();
}

real_t DistanceJoint3D::get_param(Param p_param) const {
	ERR_FAIL_INDEX_V(p_param, PARAM_MAX, 0);
	return params[p_param];
}

void DistanceJoint3D::_configure_joint(RID p_joint, PhysicsBody3D *body_a, PhysicsBody3D *body_b) {
	Transform3D gt = get_global_transform();
	Transform3D ainv = body_a->get_global_transform().affine_inverse();

	Transform3D local_a = ainv * gt;
	local_a.orthonormalize();

	Transform3D local_b = gt;
	if (body_b) {
		Transform3D binv = body_b->get_global_transform().affine_inverse();
		local_b = binv * gt;
	}
	local_b.orthonormalize();

	PhysicsServer3D::get_singleton()->joint_make_distance(p_joint, body_a->get_rid(), local_a, body_b ? body_b->get_rid() : RID(), local_b);
	for (int i = 0; i < PARAM_MAX; i++) {
		PhysicsServer3D::get_singleton()->distance_joint_set_param(p_joint, PhysicsServer3D::DistanceJointParam(i), params[i]);
	}
}

DistanceJoint3D::DistanceJoint3D() {
	params[PARAM_LIMIT_UPPER] = -1.0f;
	params[PARAM_LIMIT_LOWER] = -1.0f;
}
