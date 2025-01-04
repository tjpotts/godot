#ifndef DISTANCE_JOINT_3D_H
#define DISTANCE_JOINT_3D_H

#include "scene/3d/physics/joints/joint_3d.h"

class DistanceJoint3D : public Joint3D {
	GDCLASS(DistanceJoint3D, Joint3D);

public:
	enum Param {
		PARAM_LIMIT_UPPER = PhysicsServer3D::DISTANCE_JOINT_LIMIT_UPPER,
		PARAM_LIMIT_LOWER = PhysicsServer3D::DISTANCE_JOINT_LIMIT_LOWER,
		PARAM_MAX = PhysicsServer3D::DISTANCE_JOINT_MAX
	};

protected:
	real_t params[PARAM_MAX];
	virtual void _configure_joint(RID p_joint, PhysicsBody3D *body_a, PhysicsBody3D *body_b) override;
	static void _bind_methods();

public:
	void set_param(Param p_param, real_t p_value);
	real_t get_param(Param p_param) const;

	DistanceJoint3D();
};

VARIANT_ENUM_CAST(DistanceJoint3D::Param);

#endif // DISTANCE_JOINT_3D_H
