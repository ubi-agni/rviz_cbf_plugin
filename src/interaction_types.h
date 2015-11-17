#pragma once

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <boost/function.hpp>

#include <moveit/macros/class_forward.h>
namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
}}
namespace rviz_cbf_plugin
{

enum InteractionType {
	NONE = 0,
	TX = 1,
	TY = 2,
	TZ = 4,
	RX = 8,
	RY = 16,
	RZ = 32,

	POSITION = TX | TY | TZ,
	ORIENTATION = RX | RY | RZ,
	ALL = POSITION | ORIENTATION,
};

typedef boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)> FeedbackFn;
typedef boost::function<geometry_msgs::Pose()> GetPoseFn;

/// description of a marker attached to a link
struct LinkMarker
{
	LinkMarker(const std::string &link, InteractionType type, FeedbackFn cb)
	   : link(link), type(type), feedback_cb(cb) {}

	/// the link this marker is attached to
	std::string link;

	/// interaction directions to allow
	InteractionType type;

	/// feedback callback
	FeedbackFn feedback_cb;
};

/// description of a marker attached to a joint
struct JointMarker
{
	JointMarker(const std::string &joint, FeedbackFn cb)
	   : joint(joint), feedback_cb(cb) {}

	/// the joint this marker is associated with
	std::string joint;
	/// feedback callback
	FeedbackFn feedback_cb;
};

/// description of a generic marker
struct GenericMarker
{
	GenericMarker(const FeedbackFn & feedback_cb, const GetPoseFn &pose_cb)
	   : feedback_cb(feedback_cb), pose_cb(pose_cb) {}

	visualization_msgs::InteractiveMarker imarker;

	/// feedback callback
	FeedbackFn feedback_cb;

	/// pose callback
	GetPoseFn pose_cb;
};

} // namespace rviz_cbf_plugin
