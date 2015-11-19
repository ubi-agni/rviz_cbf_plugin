#pragma once

#include <visualization_msgs/InteractiveMarker.h>
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

	MOVE_AXIS = TX,
	ROTATE_AXIS = RX,
	MOVE_PLANE = TY | TZ,
	MOVE_ROTATE = MOVE_PLANE | RX,

	MOVE_3D = TX | TY | TZ,
	ROTATE_3D = RX | RY | RZ,
	MOVE_ROTATE_3D = MOVE_3D | ROTATE_3D,
};

typedef boost::function<void(const geometry_msgs::Pose&)> PoseFeedbackFn;
typedef boost::function<geometry_msgs::Pose()> GetMarkerPoseFn;

/// description of a marker attached to a link
struct LinkMarker
{
	LinkMarker(const std::string &link, InteractionType type, PoseFeedbackFn cb)
	   : link(link), type(type), feedback_cb(cb) {}

	/// the link this marker is attached to
	std::string link;

	/// interaction directions to allow
	InteractionType type;

	/// feedback callback
	PoseFeedbackFn feedback_cb;
};

/// description of a marker attached to a joint
struct JointMarker
{
	JointMarker(const std::string &joint, PoseFeedbackFn cb)
	   : joint(joint), feedback_cb(cb) {}

	/// the joint this marker is associated with
	std::string joint;
	/// feedback callback
	PoseFeedbackFn feedback_cb;
};

/// description of a generic marker
struct GenericMarker
{
	GenericMarker(const PoseFeedbackFn & feedback_cb, const GetMarkerPoseFn &pose_cb)
	   : feedback_cb(feedback_cb), pose_cb(pose_cb) {}

	visualization_msgs::InteractiveMarker imarker;

	/// feedback callback
	PoseFeedbackFn feedback_cb;

	/// pose callback
	GetMarkerPoseFn pose_cb;
};

} // namespace rviz_cbf_plugin
