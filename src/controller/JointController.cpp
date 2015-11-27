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

	for (size_t i=0, end=joints_.size(); i != end; ++i) {
		result.push_back(JointMarker(joints_[i].getName(),
		                             boost::bind(&JointController::markerCallback, this, _1, i)));
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

void JointController::markerCallback(const geometry_msgs::Pose &pose, unsigned int joint_id) const
{
	// TODO: use your code to compute the joint pos/angle from pose
	double joint_pos = 0;
	tf::Pose jp_tmp1, jp_tmp2;
	tf::poseMsgToTF(pose, jp_tmp1);
	tf::poseKDLToTF(joints_.at(joint_id).pose(0), jp_tmp2);
	joint_pos = jp_tmp1.getRotation().angle(jp_tmp2.getRotation());

	const_cast<JointController*>(this)->setTarget(joint_id, joint_pos);
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
	jnt_potential->set_max_gradient_step_norm(angles::from_degrees(1.) / nJoints);

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
