#include "PositionController.h"
#include "properties.h"

#include <rviz/properties/editable_enum_property.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

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

namespace rviz_cbf_plugin
{

PositionController::PositionController(const Controller &parent,
                                       const QString& name) :
   Controller(name, parent),
   target_(boost::make_shared<CBF::DummyReference>(1,3))
{
	link_name_property_ = new LinkNameProperty("Link name", "", "controlled link name",
	                                           this, SLOT(changedLinkName()), this);
	connect(&getRoot(), SIGNAL(robotModelChanged(moveit::core::RobotModelConstPtr)),
	        this, SLOT(setRobotModel(moveit::core::RobotModelConstPtr)));
}

std::list<LinkMarker> PositionController::getLinkMarkers() const
{
	auto result = Controller::getLinkMarkers(); // fetch markers from children
	result.push_back(LinkMarker(link_name_, InteractionType::MOVE_3D,
	                            boost::bind(&PositionController::markerCallback, this, _1)));
	return result;
}

static
unsigned int computeDepthFromRoot(const moveit::core::LinkModel *link) {
	unsigned int result = 0;
	while (link) {
		++result;
		link = link->getParentLinkModel();
	}
	return result;
}

void PositionController::setRobotModel(const robot_model::RobotModelConstPtr &rm)
{
	link_name_property_->setRobotModel(rm);
	if (!rm->hasLinkModel(link_name_)) {
		if (link_name_.empty()) {
			// search for end-effector that is furthest away from root joint
			const auto links = rm->getLinkModels();
			const moveit::core::LinkModel *eef = NULL;
			unsigned int maxDepth = 0;
			BOOST_FOREACH(const moveit::core::LinkModel *link, links) {
				if (link->getChildJointModels().empty()) {
					unsigned int depth = computeDepthFromRoot(link);
					if (depth > maxDepth || eef == NULL) {
						eef = link;
						maxDepth = depth;
					}
				}
			}
			if (eef) setLink(eef->getName());
			// TODO else setStatus(Error, "link", "link not found");
		}
	}

	initController();
}

void PositionController::changedLinkName()
{
	setLink(link_name_property_->getStdString());
}

void PositionController::setLink(const std::string &name)
{
	if (name == link_name_) return;

	link_name_ = name;
	link_name_property_->setValue(QString::fromStdString(link_name_));

	getRoot().emitMarkersChanged();
}

void PositionController::markerCallback(const geometry_msgs::Pose &pose) const
{
	Eigen::Affine3d tm;
	tf::poseMsgToEigen(pose, tm);
	const_cast<PositionController*>(this)->setTarget(tm.translation());
}

void PositionController::initController()
{
	boost::mutex::scoped_lock lock(controller_mutex_);

	const KDL::Tree& tree = getRoot().getKDLTree();
	KDL::Chain chain;
	tree.getChain(tree.getRootSegment()->first, link_name_, chain);
	chain_ = boost::make_shared<KDL::Chain>(chain);
	unsigned int nJoints = chain_->getNrOfJoints();

	joints_ = boost::make_shared<CBF::DummyResource>(nJoints);
	auto solver = boost::make_shared<CBF::ThresholdGenericEffectorTransform>(3, nJoints);
	solver->setThreshold(0.1);
	controller_ = boost::make_shared<CBF::PrimitiveController>
	              (1.0,
	               boost::assign::list_of(boost::make_shared<CBF::TaskSpaceDistanceThreshold>(1e-3)),
	               target_,
	               boost::make_shared<CBF::SquarePotential>(3, 1.),
	               boost::make_shared<CBF::KDLChainPositionSensorTransform>(chain_),
	               solver,
	               std::vector<CBF::SubordinateControllerPtr>(),
	               boost::make_shared<CBF::AddingStrategy>(),
	               joints_
	               );
}

void PositionController::setTarget(const Eigen::Vector3d &position)
{
	boost::mutex::scoped_lock lock(controller_mutex_);
	target_->set_reference(position);
}

static void updateResource(CBF::DummyResourcePtr joints,
                           const moveit::core::RobotStatePtr &rs,
                           KDL::Chain& chain)
{
	for (int i=0, j=0; i < chain.getNrOfSegments(); ++i) {
		const KDL::Joint &joint = chain.getSegment(i).getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		joints->m_Variables[j++] = rs->getVariablePosition(joint.getName());
	}
}

static void updateRobotState(const moveit::core::RobotStatePtr &rs,
                             CBF::DummyResourcePtr joints,
                             KDL::Chain& chain) {
	for (int i=0, j=0; i < chain.getNrOfSegments(); ++i) {
		const KDL::Joint &joint = chain.getSegment(i).getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		rs->setVariablePosition(joint.getName(), joints->m_Variables[j++]);
	}
}

void PositionController::step(const moveit::core::RobotStatePtr &rs)
{
	updateResource(joints_, rs, *chain_);
	// perform controller step
	unsigned int iMaxIter = 10;
	while (iMaxIter && controller_->step() == false) {
		--iMaxIter;
	}
	updateRobotState(rs, joints_, *chain_);
}


} // namespace rviz_cbf_plugin
