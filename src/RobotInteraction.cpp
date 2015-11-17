#include "RobotInteraction.h"
#include "marker_helpers.h"

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>

namespace vm = visualization_msgs;
namespace rm = moveit::core;

namespace rviz_cbf_plugin
{

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

struct RobotInteraction::MarkerDescription
{
	MarkerDescription(const std::string &name) {
		imarker_.name = name;
		type_ = NONE;
	}

	vm::InteractiveMarker imarker_;
	unsigned int type_;
	std::vector<FeedbackFn> feedback_cbs_;
	GetPoseFn pose_cb_;
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

void RobotInteraction::clearMarkerList()
{
	markers_.clear();
}

geometry_msgs::Pose RobotInteraction::getLinkPose(const std::string &link)
{
	geometry_msgs::Pose result;
	tf::poseEigenToMsg(robot_state_->getGlobalLinkTransform(link), result);
	return result;
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
		MarkerDescriptionPtr marker = getOrCreateMarkerDescription("LL_" + m.link);
		marker->imarker_.header.frame_id = m.link;
		marker->type_ |= m.type;
		marker->feedback_cbs_.push_back(m.feedback_cb);
		marker->pose_cb_ = boost::bind(&RobotInteraction::getLinkPose, this, m.link);
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

		MarkerDescriptionPtr marker = getOrCreateMarkerDescription("JJ_" + m.joint);
		const std::string &link_name = joint->getChildLinkModel()->getName();
		marker->imarker_.header.frame_id = link_name;
		marker->feedback_cbs_.push_back(m.feedback_cb);
		marker->pose_cb_ = boost::bind(&RobotInteraction::getLinkPose, this, link_name);
	}
}

/// create marker descriptions from @param markers
void RobotInteraction::addMarkers(const std::list<GenericMarker> &markers)
{
	BOOST_FOREACH(const GenericMarker &m, markers) {
		MarkerDescriptionPtr marker = getUniqueMarkerDescription("GG_" + m.imarker.name);
		std::string marker_name = marker->imarker_.name;
		marker->imarker_ = m.imarker;
		marker->imarker_.name = marker_name;

		marker->feedback_cbs_.push_back(m.feedback_cb);
		marker->pose_cb_ = m.pose_cb;
	}
}

/// create controls for a link
void RobotInteraction::createLinkControls(MarkerDescriptionPtr &desc)
{
	// TODO link control, select interaction mode from desc->type
	desc->imarker_.controls.empty();
	marker_helpers::addPositionControls(desc->imarker_, AXES::ALL);
	marker_helpers::addOrientationControls(desc->imarker_, AXES::ALL);
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

void RobotInteraction::processFeedback(const vm::InteractiveMarkerFeedbackConstPtr &feedback)
{
	MarkerDescriptionPtr marker = getMarkerDescription(feedback->marker_name);
	if (!marker) {
		ROS_WARN_STREAM("unknown marker " << feedback->marker_name);
		return;
	}
	BOOST_FOREACH(auto cb, marker->feedback_cbs_) {
		cb(feedback);
	}
}

} // namespace rviz_cbf_plugin
