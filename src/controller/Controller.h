#pragma once
#include "interaction_types.h"

#include <rviz/properties/property.h>
#include <map>
#include <string>

#include <moveit/macros/class_forward.h>
namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}}
namespace rviz_cbf_plugin
{

class RootController;
MOVEIT_CLASS_FORWARD(RobotInteraction);
class Controller : public rviz::Property
{
	Q_OBJECT
	friend class RootController;
public:
	explicit Controller(const QString &name, const Controller &parent);

	const Controller* getParent() const {return parent_;}
	const RootController& getRoot() const {return root_;}

	virtual std::list<LinkMarker> getLinkMarkers() const;
	virtual std::list<JointMarker> getJointMarkers() const;
	virtual std::list<GenericMarker> getGenericMarkers() const;

	template<class Type> QList<Type*> getChildren() const;

private:
	explicit Controller(const QString &name, rviz::Property *parent, const RootController &root);

private:
	Controller     const *parent_;
	const RootController &root_;
};

class RootController : public Controller
{
	Q_OBJECT

public:
	explicit RootController(rviz::Property *parent, RobotInteractionPtr &ri);
	void emitMarkersChanged() const;

Q_SIGNALS:
	void robotModelChanged(const moveit::core::RobotModelConstPtr &rm);
	void markersChanged() const;

public Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);

private:
	RobotInteractionPtr ri_;
};

template<class Type>
QList<Type*> Controller::getChildren() const {
	QList<Type*> result;
	for(int i = 0; i < this->numChildren(); ++i) {
		Type *obj = qobject_cast<Type*>(this->childAtUnchecked(i));
		if (obj) result.push_back(obj);
	}
	return result;
}

} // namespace rviz_cbf_plugin
