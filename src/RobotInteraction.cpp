#include "RobotInteraction.h"
#include <interactive_markers/interactive_marker_server.h>

namespace rviz_cbf_plugin
{

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

RobotInteraction::RobotInteraction(const robot_model::RobotModelConstPtr &robot_model,
                                   const std::string &ns) :
   robot_model_(robot_model)
{
	topic_ = ns.empty() ? INTERACTIVE_MARKER_TOPIC : ns + "/" + INTERACTIVE_MARKER_TOPIC;
	ims_ = new interactive_markers::InteractiveMarkerServer(topic_);
}

} // namespace rviz_cbf_plugin
