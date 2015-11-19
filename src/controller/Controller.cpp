#include "Controller.h"
#include "RobotInteraction.h"
#include <boost/foreach.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <cbf/primitive_controller.h>
#include <cbf/dummy_resource.h>

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

std::list<LinkMarker> Controller::getLinkMarkers() const
{
	std::list<LinkMarker> result;
	BOOST_FOREACH(const Controller* child, getChildren<Controller>()) {
		// append child list to result
		result.splice(result.end(), child->getLinkMarkers());
	}
	return result;
}
std::list<JointMarker> Controller::getJointMarkers() const
{
	std::list<JointMarker> result;
	BOOST_FOREACH(const Controller* child, getChildren<Controller>()) {
		// append child list to result
		result.splice(result.end(), child->getJointMarkers());
	}
	return result;
}
std::list<GenericMarker> Controller::getGenericMarkers() const
{
	std::list<GenericMarker> result;
	BOOST_FOREACH(const Controller* child, getChildren<Controller>()) {
		// append child list to result
		result.splice(result.end(), child->getGenericMarkers());
	}
	return result;
}


RootController::RootController(rviz::Property *parent,
                               RobotInteractionPtr &ri) :
   Controller("Controllers", parent, *this),
   ri_(ri)
{
}

void RootController::emitMarkersChanged() const
{
	Q_EMIT markersChanged();
}

void RootController::setRobotModel(const moveit::core::RobotModelConstPtr &rm)
{
	if (!kdl_parser::treeFromUrdfModel(*rm->getURDF(), kdl_tree_)) {
		// TODO: proper error handling
		return;
	}

	Q_EMIT robotModelChanged(rm);
	Q_EMIT markersChanged();
}

CBF::PrimitiveControllerPtr RootController::getController()
{
	auto children = getChildren<Controller>();
	if (children.empty()) return CBF::PrimitiveControllerPtr();
	else return children.front()->getController();
}

void RootController::step(const moveit::core::RobotStatePtr &rs)
{
	auto children = getChildren<Controller>();
	if (children.empty()) return;
	children.front()->step(rs);
}

} // namespace rviz_cbf_plugin
