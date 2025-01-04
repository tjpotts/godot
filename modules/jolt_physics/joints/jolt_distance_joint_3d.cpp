#include "jolt_distance_joint_3d.h"

#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_body_3d.h"
#include "../spaces/jolt_space_3d.h"

#include "Jolt/Physics/Constraints/DistanceConstraint.h"

JoltDistanceJoint3D::JoltDistanceJoint3D(const JoltJoint3D &p_old_joint, JoltBody3D *p_body_a, JoltBody3D *p_body_b, const Transform3D &p_local_ref_a, const Transform3D &p_local_ref_b) :
		JoltJoint3D(p_old_joint, p_body_a, p_body_b, p_local_ref_a, p_local_ref_b) {
	rebuild();
}

JPH::Constraint *JoltDistanceJoint3D::_build_distance(JPH::Body *p_jolt_body_a, JPH::Body *p_jolt_body_b, const Transform3D &p_shifted_ref_a, const Transform3D &p_shifted_ref_b, float min_distance, float max_distance) const {
	JPH::DistanceConstraintSettings constraint_settings;

	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = to_jolt_r(p_shifted_ref_a.origin);
	constraint_settings.mPoint2 = to_jolt_r(p_shifted_ref_b.origin);
	constraint_settings.mMinDistance = min_distance;
	constraint_settings.mMaxDistance = max_distance;

	if (p_jolt_body_a == nullptr) {
		return constraint_settings.Create(JPH::Body::sFixedToWorld, *p_jolt_body_b);
	} else if (p_jolt_body_b == nullptr) {
		return constraint_settings.Create(*p_jolt_body_a, JPH::Body::sFixedToWorld);
	} else {
		return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
	}
}

double JoltDistanceJoint3D::get_param(Parameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::DISTANCE_JOINT_LIMIT_UPPER: {
			return limit_upper;
		}
		case PhysicsServer3D::DISTANCE_JOINT_LIMIT_LOWER: {
			return limit_lower;
		}
		default: {
			ERR_FAIL_V_MSG(0.0, vformat("Unhandled parameter: '%d'. This should not happen. Please report this.", p_param));
		}
	}
}

void JoltDistanceJoint3D::set_param(Parameter p_param, double p_value) {
	switch (p_param) {
		case PhysicsServer3D::DISTANCE_JOINT_LIMIT_UPPER: {
			limit_upper = p_value;
			rebuild();
		} break;
		case PhysicsServer3D::DISTANCE_JOINT_LIMIT_LOWER: {
			limit_lower = p_value;
			rebuild();
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled parameter: '%d'. This should not happen. Please report this.", p_param));
		} break;
	}
}

void JoltDistanceJoint3D::rebuild() {
	destroy();

	JoltSpace3D *space = get_space();

	if (space == nullptr) {
		return;
	}

	const JPH::BodyID body_ids[2] = {
		body_a != nullptr ? body_a->get_jolt_id() : JPH::BodyID(),
		body_b != nullptr ? body_b->get_jolt_id() : JPH::BodyID()
	};

	const JoltWritableBodies3D jolt_bodies = space->write_bodies(body_ids, 2);

	JPH::Body *jolt_body_a = static_cast<JPH::Body *>(jolt_bodies[0]);
	JPH::Body *jolt_body_b = static_cast<JPH::Body *>(jolt_bodies[1]);

	ERR_FAIL_COND(jolt_body_a == nullptr && jolt_body_b == nullptr);

	Transform3D shifted_ref_a;
	Transform3D shifted_ref_b;

	jolt_ref = _build_distance(jolt_body_a, jolt_body_b, shifted_ref_a, shifted_ref_b);

	space->add_joint(this);
}
