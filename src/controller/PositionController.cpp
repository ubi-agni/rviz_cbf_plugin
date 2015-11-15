#include "PositionController.h"
#include <rviz/properties/editable_enum_property.h>
#include <moveit/robot_model/robot_model.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

namespace rviz_cbf_plugin
{

PositionController::PositionController(const Controller &parent) :
   Controller("position", parent)
{
	link_name_property_ = new LinkNameProperty("Link name", "", "controlled link name",
	                                                 this, SLOT(changedLinkName()), this);
	connect(&getRoot(), SIGNAL(robotModelChanged(robot_model::RobotModelConstPtr)),
	        this, SLOT(setRobotModel(robot_model::RobotModelConstPtr)));
// TODO	addPositionMarker(link_name_, boost::bind(&PositionController::markerCallback, this, _1));
}

void PositionController::setRobotModel(const robot_model::RobotModelConstPtr &rm)
{
	link_name_property_->setRobotModel(rm);
	if (!rm->getLinkModel(link_name_)) {
		if (link_name_.empty()) {
			// use first end effector as default
			const auto links = rm->getLinkModels();
			BOOST_FOREACH(const moveit::core::LinkModel *link, links) {
				if (link->getChildJointModels().empty()) {
					link_name_property_->setString(QString::fromStdString(link->getName()));
					break;
				}
			}
		}
	}
}

void PositionController::changedLinkName()
{

}

void PositionController::markerCallback(const Eigen::Vector3d &position)
{
// TODO	target_->set_reference(position);
}

} // namespace rviz_cbf_plugin
