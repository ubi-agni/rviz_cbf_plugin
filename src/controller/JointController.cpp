#include "JointController.h"
#include "properties.h"

#include <rviz/properties/editable_enum_property.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

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
	// TODO: push markers for *all* joints
	BOOST_FOREACH(std::string joint_name, joint_list_) {
		std::cout<<"JJ_" + joint_name<<std::endl;
		result.push_back(JointMarker(joint_name, boost::bind(&JointController::markerCallback, this, _1)));
	}
	return result;
}

static
unsigned int computeDepthFromRoot(const moveit::core::JointModel *joint) {
	unsigned int result = 0;
	const moveit::core::LinkModel* link = joint->getParentLinkModel();
	while (link) {
		++result;
		link = link->getParentLinkModel();
	}
	return result;
}

void JointController::setRobotModel(const robot_model::RobotModelConstPtr &rm)
{
	joint_list_ = rm->getJointModelNames();

	BOOST_FOREACH(std::string joint_name, joint_list_) {
		std::cout<<joint_name<<std::endl;
	}
	initController();
}

void JointController::markerCallback(const geometry_msgs::Pose &pose) const
{
	Eigen::Affine3d tm;
	tf::poseMsgToEigen(pose, tm);
}

void JointController::initController()
{
	boost::mutex::scoped_lock lock(controller_mutex_);

	const KDL::Tree& tree = getRoot().getKDLTree();
	unsigned int nJoints = tree.getNrOfJoints();

	std::vector<CBF::ConvergenceCriterionPtr> convergence = boost::assign::list_of
		(boost::make_shared<CBF::TaskSpaceDistanceThreshold>(1e-3));
	CBF::PotentialPtr jnt_potential(new CBF::SquarePotential(nJoints, 1.));
	jnt_potential->set_max_gradient_step_norm(angles::from_degrees(1.) / nJoints);

	joints_ = boost::make_shared<CBF::DummyResource>(nJoints);
	auto solver = boost::make_shared<CBF::IdentityEffectorTransform>(nJoints);
	controller_ = boost::make_shared<CBF::PrimitiveController>
	              (1.0,
	               convergence,
	               boost::make_shared<CBF::DummyReference>(1,nJoints),
	               jnt_potential,
	               boost::make_shared<CBF::IdentitySensorTransform>(nJoints),
	               solver,
	               std::vector<CBF::SubordinateControllerPtr>(),
	               boost::make_shared<CBF::AddingStrategy>(),
	               joints_
	               );
}

static void updateResource(CBF::DummyResourcePtr joints,
                           const moveit::core::RobotStatePtr &rs,
                           const KDL::Tree& tree)
{
	unsigned int j=0;
	for (auto it = tree.getSegments().begin(),
	     end = tree.getSegments().end(); it != end; ++it) {
		const KDL::Joint &joint = it->second.segment.getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		joints->m_Variables[j++] = rs->getVariablePosition(joint.getName());
	}
}

static void updateRobotState(const moveit::core::RobotStatePtr &rs,
                             CBF::DummyResourcePtr joints,
                             const KDL::Tree& tree) {
	unsigned int j=0;
	for (auto it = tree.getSegments().begin(),
	     end = tree.getSegments().end(); it != end; ++it) {
		const KDL::Joint &joint = it->second.segment.getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		rs->setVariablePosition(joint.getName(), joints->m_Variables[j++]);
	}
}

void JointController::step(const moveit::core::RobotStatePtr &rs)
{
	updateResource(joints_, rs, getRoot().getKDLTree());
	// perform controller step
	unsigned int iMaxIter = 10;
	while (iMaxIter && controller_->step() == false) {
		--iMaxIter;
	}
	updateRobotState(rs, joints_, getRoot().getKDLTree());
}


} // namespace rviz_cbf_plugin
