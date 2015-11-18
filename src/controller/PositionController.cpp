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

std::list<LinkMarker> PositionController::getLinkMarkers() const
{
	auto result = Controller::getLinkMarkers(); // fetch markers from children
	result.push_back(LinkMarker(link_name_, ALL,
	                            boost::bind(&PositionController::markerCallback, this, _1)));
	return result;
}

static
unsigned int computeDepthFromRoot(const moveit::core::LinkModel *link) {
	unsigned int result = 0;
	while (link) {
		++result;
		link = link->getParentLinkModel();
	}
	return result;
}

void PositionController::setRobotModel(const robot_model::RobotModelConstPtr &rm)
{
	link_name_property_->setRobotModel(rm);
	if (!rm->hasLinkModel(link_name_)) {
		if (link_name_.empty()) {
			// search for end-effector that is furthest away from root joint
			const auto links = rm->getLinkModels();
			const moveit::core::LinkModel *eef = NULL;
			unsigned int maxDepth = 0;
			BOOST_FOREACH(const moveit::core::LinkModel *link, links) {
				if (link->getChildJointModels().empty()) {
					unsigned int depth = computeDepthFromRoot(link);
					if (depth > maxDepth || eef == NULL) {
						eef = link;
						maxDepth = depth;
					}
				}
			}
			if (eef) setLink(eef->getName());
			// TODO else setStatus(Error, "link", "link not found");
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

	getRoot().emitMarkersChanged();
}

void PositionController::markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr) const
{
}

void PositionController::setTarget(const Eigen::Vector3d &pos)
{
// TODO	target_->set_reference(position);
}

} // namespace rviz_cbf_plugin
