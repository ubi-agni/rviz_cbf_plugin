#include "JointController.h"
#include "properties.h"

#include <rviz/properties/editable_enum_property.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <angles/angles.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/locks.hpp>

#include <cbf/dummy_reference.h>
#include <cbf/dummy_resource.h>
#include <cbf/kdl_transforms.h>
#include <cbf/square_potential.h>
#include <cbf/generic_transform.h>
#include <cbf/identity_transform.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

namespace rviz_cbf_plugin
{

JointController::JointController(const Controller &parent,
                                 const QString& name) :
   Controller(name, parent),
   target_(boost::make_shared<CBF::DummyReference>(1,3))
{
	connect(&getRoot(), SIGNAL(robotModelChanged(moveit::core::RobotModelConstPtr)),
	        this, SLOT(setRobotModel(moveit::core::RobotModelConstPtr)));
}

std::list<JointMarker> JointController::getJointMarkers() const
{
	auto result = Controller::getJointMarkers(); // fetch markers from children
	const KDL::Tree &tree = getRoot().getKDLTree();

	PoseFeedbackFn cb;
	for (size_t i=0, end=joints_.size(); i != end; ++i) {
		const KDL::Joint &joint = joints_[i];
		switch (joint.getType()) {
		case KDL::Joint::TransAxis:
		case KDL::Joint::TransX:
		case KDL::Joint::TransY:
		case KDL::Joint::TransZ:
			cb = boost::bind(&JointController::computePrismaticTarget, this, _1,
			                 Eigen::Vector3d(joint.JointAxis().data), i);
			break;

		case KDL::Joint::RotAxis:
		case KDL::Joint::RotX:
		case KDL::Joint::RotY:
		case KDL::Joint::RotZ:
			cb = boost::bind(&JointController::computeRevoluteTarget, this, _1,
			                 Eigen::Vector3d(joint.JointAxis().data), i);
			break;
		}

		result.push_back(JointMarker(joint.getName(), cb));
	}
	return result;
}

void JointController::setRobotModel(const robot_model::RobotModelConstPtr &rm)
{
	initController();

	// (re)create active joints_ vector
	joints_.clear();
	const KDL::Tree &tree = getRoot().getKDLTree();
	for (auto it = tree.getSegments().begin(),
	     end = tree.getSegments().end(); it != end; ++it) {
		const KDL::Joint &joint = it->second.segment.getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		joints_.push_back(joint);
	}
}

void JointController::computePrismaticTarget(const geometry_msgs::Pose &pose, const Eigen::Vector3d &axis,
                                             unsigned int joint_id) const
{
	const geometry_msgs::Point &p = pose.position;
	Eigen::Vector3d pos(p.x, p.y, p.z);
	const_cast<JointController*>(this)->setTarget(joint_id, pos.dot(axis));
}

void JointController::computeRevoluteTarget(const geometry_msgs::Pose &pose, const Eigen::Vector3d &axis,
                                            unsigned int joint_id) const
{
	/* simple computation from acos(quat.w) doesn't work, as the quaternion computation
	 * from matrix already restricts the joint angle to [0..pi], because of acos((tr(R) - 1) / 2) in [0..pi].
	 * The rotation axis is accordingly flipped if neccessary.
	 * Hence, use atan2 to compute the angle - considering both, cos and sin of angle */
	const geometry_msgs::Quaternion &o = pose.orientation;
	Eigen::Quaterniond q(o.w, o.x, o.y, o.z); q.normalize();
	size_t maxIdx;
	axis.array().abs().maxCoeff(&maxIdx);
	const_cast<JointController*>(this)->setTarget(joint_id, 2.*atan2(q.vec()[maxIdx] / axis[maxIdx], q.w()));
}

void JointController::initController()
{
	boost::mutex::scoped_lock lock(controller_mutex_);

	const KDL::Tree& tree = getRoot().getKDLTree();
	unsigned int nJoints = tree.getNrOfJoints();

	target_ = boost::make_shared<CBF::DummyReference>(1,nJoints);

	std::vector<CBF::ConvergenceCriterionPtr> convergence = boost::assign::list_of
		(boost::make_shared<CBF::TaskSpaceDistanceThreshold>(1e-3));

	CBF::PotentialPtr jnt_potential(new CBF::SquarePotential(nJoints, 1.));
	jnt_potential->set_max_gradient_step_norm(angles::from_degrees(360) / nJoints);

	joint_values_ = boost::make_shared<CBF::DummyResource>(nJoints);
	auto solver = boost::make_shared<CBF::IdentityEffectorTransform>(nJoints);
	controller_ = boost::make_shared<CBF::PrimitiveController>
	              (1.0,
	               convergence,
	               target_,
	               jnt_potential,
	               boost::make_shared<CBF::IdentitySensorTransform>(nJoints),
	               solver,
	               std::vector<CBF::SubordinateControllerPtr>(),
	               boost::make_shared<CBF::AddingStrategy>(),
	               joint_values_
	               );
}

void JointController::setTarget(unsigned int joint_id, double joint_pos)
{
	boost::mutex::scoped_lock lock(controller_mutex_);
	target_->get()[0][joint_id] = joint_pos;
}

static void updateResource(CBF::DummyResourcePtr joint_values,
                           const moveit::core::RobotStatePtr &rs,
                           const std::vector<KDL::Joint>& joints)
{
	for (size_t i=0, end=joints.size(); i != end; ++i) {
		joint_values->m_Variables[i] = rs->getVariablePosition(joints[i].getName());
	}
}

static void updateRobotState(const moveit::core::RobotStatePtr &rs,
                             CBF::DummyResourcePtr joint_values,
                             const std::vector<KDL::Joint>& joints) {
	for (size_t i=0, end=joints.size(); i != end; ++i) {
		rs->setVariablePosition(joints[i].getName(), joint_values->m_Variables[i]);
	}
}

void JointController::step(const moveit::core::RobotStatePtr &rs)
{
	boost::mutex::scoped_lock lock(controller_mutex_);

	updateResource(joint_values_, rs, joints_);
	// perform controller step
	unsigned int iMaxIter = 10;
	while (iMaxIter && controller_->step() == false) {
		--iMaxIter;
	}
	updateRobotState(rs, joint_values_, joints_);
}


} // namespace rviz_cbf_plugin
