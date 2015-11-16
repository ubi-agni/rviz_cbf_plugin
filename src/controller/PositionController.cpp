#include "PositionController.h"
#include "properties.h"

#include <rviz/properties/editable_enum_property.h>
#include <moveit/robot_model/robot_model.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

namespace rviz_cbf_plugin
{

PositionController::PositionController(const Controller &parent,
                                       const QString& name) :
   Controller(name, parent)
{
	link_name_property_ = new LinkNameProperty("Link name", "", "controlled link name",
	                                           this, SLOT(changedLinkName()), this);
	connect(&getRoot(), SIGNAL(robotModelChanged(moveit::core::RobotModelConstPtr)),
	        this, SLOT(setRobotModel(moveit::core::RobotModelConstPtr)));
}

void PositionController::setRobotModel(const robot_model::RobotModelConstPtr &rm)
{
	link_name_property_->setRobotModel(rm);
	if (!rm->getLinkModel(link_name_)) {
		if (link_name_.empty()) {
			// use first end effector as default
			const auto links = rm->getLinkModels();
			BOOST_FOREACH(const moveit::core::LinkModel *link, links) {
				// TODO: fix finding end effector.
				if (link->getChildJointModels().empty()) {
					setLink(link->getName());
					break;
				}
			}
		}
	}
}

void PositionController::changedLinkName()
{
	setLink(link_name_property_->getStdString());
}

void PositionController::setLink(const std::string &name)
{
	if (name == link_name_) return;

	link_name_ = name;
	link_name_property_->setValue(QString::fromStdString(link_name_));

	emit linkNameChanged(link_name_);

	// TODO	addPositionMarker(link_name_, boost::bind(&PositionController::markerCallback, this, _1));
}

void PositionController::markerCallback(const Eigen::Vector3d &position)
{
// TODO	target_->set_reference(position);
}

} // namespace rviz_cbf_plugin
