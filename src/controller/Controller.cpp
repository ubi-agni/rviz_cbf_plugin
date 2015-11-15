#include "Controller.h"
#include "RootController.h"

namespace rviz_cbf_plugin
{

Controller::Controller(const QString &name, const Controller *parent) :
   rviz::Property(name, QVariant(), "", const_cast<Controller*>(parent))
{
}

void Controller::showMarkers(bool bShow) const
{
	root_->showMarkers(markers_);
}

void Controller::hideMarkers(bool bHide) const
{
	root_->hideMarkers(markers_);
}

} // namespace rviz_cbf_plugin
