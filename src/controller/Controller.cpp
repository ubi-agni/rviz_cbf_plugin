#include "Controller.h"
#include "RobotInteraction.h"

namespace rviz_cbf_plugin
{

Controller::Controller(const QString &name, const Controller &parent)
   : rviz::Property(name, QVariant(), "", const_cast<Controller*>(&parent)),
     parent_(&parent), root_(parent.getRoot())
{
}

Controller::Controller(const QString &name, rviz::Property *parent,
                       const RootController &root)
   : rviz::Property(name, QVariant(), "", parent),
	parent_(0), root_(root)
{
}

void Controller::showMarkers(bool bShow) const
{
	root_.showMarkers(markers_);
}

void Controller::hideMarkers(bool bHide) const
{
	root_.hideMarkers(markers_);
}


RootController::RootController(rviz::Property *parent,
                               RobotInteractionPtr &ri) :
   Controller("Controllers", parent, *this),
   ri_(ri)
{
}

void RootController::setRobotModel(const moveit::core::RobotModelConstPtr &rm)
{
	robotModelChanged(rm);
}

void RootController::showMarkers(const Controller::RegisteredMarkers &markers) const
{

}

void RootController::hideMarkers(const Controller::RegisteredMarkers &markers) const
{

}


} // namespace rviz_cbf_plugin
