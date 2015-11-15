#pragma once
#include "rviz/properties/property.h"
#include <Eigen/Core>

namespace rviz {
class EditableEnumProperty;
}

namespace rviz_cbf_plugin
{

class PositionController : public rviz::Property
{
	Q_OBJECT
public:
	explicit PositionController(rviz::Property *parent = 0);

protected:
	void markerCallback(const Eigen::Vector3d &position);

protected Q_SLOTS:
	void changedLinkName();

protected:
	std::string link_name_;
	rviz::EditableEnumProperty *link_name_property_;
};

} // namespace rviz_cbf_plugin
