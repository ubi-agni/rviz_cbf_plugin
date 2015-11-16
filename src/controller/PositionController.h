#pragma once
#include "Controller.h"
#include <Eigen/Core>

namespace rviz {
}

namespace rviz_cbf_plugin
{

class LinkNameProperty;
class PositionController : public Controller
{
	Q_OBJECT
public:
	explicit PositionController(const Controller &parent, const QString &name="Position");

public Q_SLOTS:
	void setLink(const std::string &name);

protected:
	void markerCallback(const Eigen::Vector3d &position);

protected Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
	void changedLinkName();

Q_SIGNALS:
	void linkNameChanged(const std::string& name);

protected:
	std::string link_name_;
	LinkNameProperty *link_name_property_;
};

} // namespace rviz_cbf_plugin
