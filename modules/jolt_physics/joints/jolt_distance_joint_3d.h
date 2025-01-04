#ifndef JOLT_DISTANCE_JOINT_3D_H
#define JOLT_DISTANCE_JOINT_3D_H

#include "jolt_joint_3d.h"

#include "../jolt_physics_server_3d.h"
#include "Jolt/Jolt.h"

#include "Jolt/Physics/Constraints/DistanceConstraint.h"

class JoltDistanceJoint3D final : public JoltJoint3D {
	typedef PhysicsServer3D::DistanceJointParam Parameter;

	double limit_lower = -1.0f;
	double limit_upper = -1.0f;

	JPH::Constraint *_build_distance(JPH::Body *p_jolt_body_a, JPH::Body *p_jolt_body_b, const Transform3D &p_shifted_ref_a, const Transform3D &p_shifted_ref_b, float min_distance = -1.0f, float max_distance = -1.0f) const;

public:
	JoltDistanceJoint3D(const JoltJoint3D &p_old_joint, JoltBody3D *p_body_a, JoltBody3D *p_body_b, const Transform3D &p_local_ref_a, const Transform3D &p_local_ref_b);

	virtual PhysicsServer3D::JointType get_type() const override { return PhysicsServer3D::JOINT_TYPE_DISTANCE; }

	double get_param(Parameter p_param) const;
	void set_param(Parameter p_param, double p_value);

	virtual void rebuild() override;
};

#endif
