#include "PositionController.h"
#include <rviz/properties/editable_enum_property.h>
#include <boost/bind.hpp>

namespace rviz_cbf_plugin
{

PositionController::PositionController(rviz::Property *parent) :
   rviz::Property("position", QVariant(), "", parent)
{
	link_name_property_ = new rviz::EditableEnumProperty("Link name", "", "controlled link name",
	                                                     this, SLOT(changedLinkName()), this);
// TODO	addPositionMarker(link_name_, boost::bind(&PositionController::markerCallback, this, _1));
}

void PositionController::changedLinkName()
{

}

void PositionController::markerCallback(const Eigen::Vector3d &position)
{
// TODO	target_->set_reference(position);
}

} // namespace rviz_cbf_plugin
