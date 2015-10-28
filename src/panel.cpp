#include "panel.h"

namespace rviz_cbf_plugin {

Panel::Panel(QWidget* parent) : rviz::Panel(parent)
{
}

void Panel::load(const rviz::Config &config)
{

}

void Panel::save(rviz::Config config) const
{

}

} // end namespace rviz_cbf_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cbf_plugin::Panel,rviz::Panel)
