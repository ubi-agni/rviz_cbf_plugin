#pragma once

#include <moveit/robot_model/robot_model.h>

namespace interactive_markers {
class InteractiveMarkerServer;
}

namespace rviz_cbf_plugin
{

class RobotInteraction
{
public:
	static const std::string INTERACTIVE_MARKER_TOPIC;

	RobotInteraction(const robot_model::RobotModelConstPtr &robot_model,
	                 const std::string &ns = "");

	const std::string& getServerTopic(void) const {return topic_;}

protected:
	robot_model::RobotModelConstPtr robot_model_;
	interactive_markers::InteractiveMarkerServer *ims_;
	std::string topic_;
};

} // namespace rviz_cbf_plugin
