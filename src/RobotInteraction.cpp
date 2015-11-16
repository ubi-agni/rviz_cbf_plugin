#include "RobotInteraction.h"
#include <interactive_markers/interactive_marker_server.h>
#include <boost/function.hpp>

namespace rviz_cbf_plugin
{

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";


RobotInteraction::RobotInteraction(const std::string &ns)
   : nextId(0)
{
	topic_ = ns.empty() ? INTERACTIVE_MARKER_TOPIC : ns + "/" + INTERACTIVE_MARKER_TOPIC;
	ims_ = new interactive_markers::InteractiveMarkerServer(topic_);
}

void RobotInteraction::setRobotModel(const moveit::core::RobotModelConstPtr &rm)
{
	robot_model_ = rm;
}

RobotInteraction::CallbackID RobotInteraction::getCallbackId()
{
	return nextId_++;
}

} // namespace rviz_cbf_plugin
