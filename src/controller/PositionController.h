#pragma once
#include "Controller.h"
#include "properties.h"
#include <Eigen/Core>

namespace rviz {
}

namespace rviz_cbf_plugin
{

class PositionController : public Controller
{
	Q_OBJECT
public:
	explicit PositionController(const Controller &parent);

protected:
	void markerCallback(const Eigen::Vector3d &position);

protected Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
	void changedLinkName();

protected:
	std::string link_name_;
	LinkNameProperty *link_name_property_;
};

} // namespace rviz_cbf_plugin
