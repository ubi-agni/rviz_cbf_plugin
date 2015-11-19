#include "RobotInteraction.h"
#include "marker_helpers.h"

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <moveit/robot_model/link_model.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>
#include <QColor>

namespace vm = visualization_msgs;
namespace rm = moveit::core;

namespace rviz_cbf_plugin
{

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

struct RobotInteraction::MarkerDescription
{
	MarkerDescription(const std::string &name) {
		imarker_.name = name;
		imarker_.scale = 0;
		type_ = NONE;
		dirty_ = false;
	}

	vm::InteractiveMarker imarker_;
	unsigned int type_; ///< interaction type
	std::string link_control_; ///< name of link to use for direct link interaction
	bool dirty_; ///< did we received a pose update?
	std::vector<PoseFeedbackFn> feedback_cbs_; ///< list of feedback callbacks
	GetMarkerPoseFn pose_cb_; ///< function pointer to retrieve updated imarker pose
};

RobotInteraction::RobotInteraction(const std::string &ns)
{
	topic_ = ns.empty() ? INTERACTIVE_MARKER_TOPIC : ns + "/" + INTERACTIVE_MARKER_TOPIC;
	ims_ = new interactive_markers::InteractiveMarkerServer(topic_);
}

void RobotInteraction::setRobotState(const moveit::core::RobotStateConstPtr &rs)
{
	robot_state_ = rs;
}

void RobotInteraction::clearMarkerDescriptions()
{
	markers_.clear();
}

geometry_msgs::Pose RobotInteraction::getLinkPose(const std::string &link)
{
	geometry_msgs::Pose result;
	tf::poseEigenToMsg(robot_state_->getGlobalLinkTransform(link), result);
	return result;
}

static const double DEFAULT_SCALE = 0.25;
double RobotInteraction::computeLinkMarkerSize(const std::string &target_link_name,
                                               std::string *actual_link_name)
{
	const rm::LinkModel *lm = robot_state_->getLinkModel(target_link_name);
	double size = 0;

	while (lm) {
		const Eigen::Vector3d &ext = lm->getShapeExtentsAtOrigin();
		size = ext.maxCoeff();
		if (size > 0) break;

		// process kinematic chain upwards (but only following fixed joints)
		// to find a link with some non-empty shape
		if (lm->getParentJointModel()->getType() == rm::JointModel::FIXED)
			lm = lm->getParentLinkModel();
		else
			lm = 0;
	}
	if (!lm) return DEFAULT_SCALE; // no link with non-zero shape extends found

	if (actual_link_name)
		*actual_link_name = lm->getName();

	return 2.01 * size;
}

// test if all bits in test are set in overset too
static
bool isSubSet(unsigned int test, unsigned int overset)
{
	return (test & overset == test);
}

static
unsigned int getInteractionMode(unsigned int interaction_types)
{
	if (isSubSet(interaction_types, MOVE_ROTATE_3D)) return vm::InteractiveMarkerControl::MOVE_ROTATE_3D;
	else if (isSubSet(interaction_types, ROTATE_3D)) return vm::InteractiveMarkerControl::ROTATE_3D;
	else if (isSubSet(interaction_types, MOVE_3D))   return vm::InteractiveMarkerControl::MOVE_3D;

	else if (isSubSet(interaction_types, MOVE_ROTATE)) return vm::InteractiveMarkerControl::MOVE_ROTATE;
	else if (isSubSet(interaction_types, MOVE_PLANE))  return vm::InteractiveMarkerControl::MOVE_PLANE;

	else if (isSubSet(interaction_types, ROTATE_AXIS)) return vm::InteractiveMarkerControl::ROTATE_AXIS;
	else if (isSubSet(interaction_types, MOVE_AXIS))   return vm::InteractiveMarkerControl::MOVE_AXIS;
	return 0;
}

/// add direct mouse control of the link geometry
bool RobotInteraction::addLinkControl(const std::string &link,
                                      unsigned int interaction_mode,
                                      vm::InteractiveMarker &im) const
{
	if (interaction_mode == 0)
		return false;
	if (!robot_state_->getRobotModel()->hasLinkModel(link))
		return false;

	vm::InteractiveMarkerControl control;
	control.interaction_mode = interaction_mode;

	visualization_msgs::MarkerArray marker_array;
	std::vector<std::string> link_names; link_names.push_back(link);
	robot_state_->getRobotMarkers(marker_array, link_names);
	if (marker_array.markers.empty()) return false;

	// compute offset transforms from link to marker
	Eigen::Affine3d tf_link_to_root = robot_state_->getGlobalLinkTransform(link).inverse();
	for(auto m = marker_array.markers.begin(), end = marker_array.markers.end();
	    m != end; ++m) {
		m->mesh_use_embedded_materials = true;
		m->header.frame_id = link;
		Eigen::Affine3d tf_root_to_mesh;
		tf::poseMsgToEigen(m->pose, tf_root_to_mesh);
		tf::poseEigenToMsg(tf_link_to_root * tf_root_to_mesh, m->pose);
		control.markers.push_back(*m);
	}

	im.controls.push_back(control);
	return true;
}

/// add mouse control for a sphere at the end effector
bool RobotInteraction::addSphereControl(unsigned int interaction_mode,
                                        visualization_msgs::InteractiveMarker &im) const
{
	if (interaction_mode == 0)
		return false;

	vm::InteractiveMarkerControl control;
	control.interaction_mode = interaction_mode;

	vm::Marker marker;
	marker.type = vm::Marker::SPHERE;
	marker.pose.orientation.w = 1;
	marker.scale.x = marker.scale.y = marker.scale.z = 0.5 * im.scale;
	QColor c("cyan");
	marker.color.r = c.redF();
	marker.color.g = c.greenF();
	marker.color.b = c.blueF();
	marker.color.a = 0.5;
	control.markers.push_back(marker);

	im.controls.push_back(control);
	return true;

}

/// return an existing marker description or nullptr
RobotInteraction::MarkerDescriptionPtr
RobotInteraction::getMarkerDescription(const std::string &name) const
{
	auto it = markers_.find(name);
	if (it != markers_.end()) return it->second;
	else return MarkerDescriptionPtr();
}
/// return an existing or create new marker description
RobotInteraction::MarkerDescriptionPtr
RobotInteraction::getOrCreateMarkerDescription(const std::string &name)
{
	auto it = markers_.find(name);
	if (it != markers_.end()) return it->second;
	return markers_[name] = boost::make_shared<MarkerDescription>(name);
}
/// return an existing marker description or nullptr
RobotInteraction::MarkerDescriptionPtr
RobotInteraction::getUniqueMarkerDescription(const std::string &name)
{
	MarkerDescriptionPtr result = getMarkerDescription(name);
	unsigned int i=0;
	while (result) {
		result = getMarkerDescription(name + boost::lexical_cast<std::string>(++i));
	}
	return getOrCreateMarkerDescription(name + boost::lexical_cast<std::string>(i));
}


/// create / augment marker descriptions from @param markers
void RobotInteraction::addMarkers(const std::list<LinkMarker> &markers)
{
	BOOST_FOREACH(const LinkMarker &m, markers) {
		const rm::LinkModel *link = robot_state_->getLinkModel(m.link);
		if (!link) {
			ROS_WARN_STREAM("no link named " << m.link);
			continue;
		}
		MarkerDescriptionPtr desc = getOrCreateMarkerDescription("LL_" + m.link);
		desc->imarker_.header.frame_id = robot_state_->getRobotModel()->getRootLinkName();
		if (desc->imarker_.scale == 0)
			desc->imarker_.scale = computeLinkMarkerSize(m.link, &desc->link_control_);
		desc->type_ |= m.type;
		desc->feedback_cbs_.push_back(m.feedback_cb);
		desc->pose_cb_ = boost::bind(&RobotInteraction::getLinkPose, this, m.link);
	}
}

/// create / augment marker descriptions from @param markers
void RobotInteraction::addMarkers(const std::list<JointMarker> &markers)
{
	BOOST_FOREACH(const JointMarker &m, markers) {
		const rm::JointModel *joint = robot_state_->getJointModel(m.joint);
		if (!joint) {
			ROS_WARN_STREAM("no joint named " << m.joint);
			continue;
		}

		MarkerDescriptionPtr desc = getOrCreateMarkerDescription("JJ_" + m.joint);
		const std::string &link_name = joint->getChildLinkModel()->getName();
		desc->imarker_.header.frame_id = link_name;
		desc->feedback_cbs_.push_back(m.feedback_cb);
		desc->pose_cb_ = boost::bind(&RobotInteraction::getLinkPose, this, link_name);
	}
}

/// create marker descriptions from @param markers
void RobotInteraction::addMarkers(const std::list<GenericMarker> &markers)
{
	BOOST_FOREACH(const GenericMarker &m, markers) {
		MarkerDescriptionPtr desc = getUniqueMarkerDescription("GG_" + m.imarker.name);
		std::string marker_name = desc->imarker_.name;
		desc->imarker_ = m.imarker;
		desc->imarker_.name = marker_name;

		desc->feedback_cbs_.push_back(m.feedback_cb);
		desc->pose_cb_ = m.pose_cb;
	}
}

/// create controls for a link
void RobotInteraction::createLinkControls(MarkerDescriptionPtr &desc)
{
	// TODO link control, select interaction mode from desc->type
	desc->imarker_.controls.empty();
	marker_helpers::addPositionControls(desc->imarker_, marker_helpers::AXES::ALL);
	marker_helpers::addOrientationControls(desc->imarker_, marker_helpers::AXES::ALL);

	unsigned int interaction_mode = getInteractionMode(desc->type_);
	addSphereControl(interaction_mode, desc->imarker_);
//	addLinkControl(desc->link_control_, interaction_mode, desc->imarker_);
}

/// create controls for a link
void RobotInteraction::createJointControls(MarkerDescriptionPtr &desc)
{
	// TODO
}

/// publish all markers_
void RobotInteraction::publishMarkers()
{
	ims_->clear();
	BOOST_FOREACH(MarkerDescriptionPtr desc, markers_ | boost::adaptors::map_values) {
		desc->imarker_.pose = desc->pose_cb_();
		if (boost::algorithm::starts_with(desc->imarker_.name, "LL_"))
			createLinkControls(desc);
		else if (boost::algorithm::starts_with(desc->imarker_.name, "LL_"))
			createJointControls(desc);
		desc->dirty_ = true;
		ims_->insert(desc->imarker_, boost::bind(&RobotInteraction::processFeedback, this, _1));
	}
	ims_->applyChanges();
}

void RobotInteraction::updateMarkerPoses()
{
	BOOST_FOREACH(const MarkerDescriptionPtr desc, markers_ | boost::adaptors::map_values) {
		ims_->setPose(desc->imarker_.name, desc->pose_cb_());
	}
	ims_->applyChanges();
}

void RobotInteraction::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	if (feedback->event_type != vm::InteractiveMarkerFeedback::POSE_UPDATE)
		return;

	MarkerDescriptionPtr desc = getMarkerDescription(feedback->marker_name);
	if (!desc) {
		ROS_WARN_STREAM("unknown marker " << feedback->marker_name);
		return;
	}
	desc->imarker_.pose = feedback->pose;
	desc->dirty_ = true;
}
void RobotInteraction::processCallbacks()
{
	// call feedback callbacks to update targets of controllers
	BOOST_FOREACH(MarkerDescriptionPtr desc, markers_ | boost::adaptors::map_values) {
		if (!desc->dirty_) continue;
		BOOST_FOREACH(auto cb, desc->feedback_cbs_)
			cb(desc->imarker_.pose);
		desc->dirty_ = false;
	}
}

} // namespace rviz_cbf_plugin
