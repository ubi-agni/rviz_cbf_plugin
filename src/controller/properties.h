#pragma once

#include <rviz/properties/editable_enum_property.h>

#include <moveit/macros/class_forward.h>
namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}}

/** @file
 *  @brief Provide some commonly used derivatives of rviz::Properties
 */

namespace rviz_cbf_plugin
{

/** Listbox showing all link names of a RobotModel */
class LinkNameProperty : public rviz::EditableEnumProperty {
	Q_OBJECT
public:
	explicit LinkNameProperty(const QString &name = "Link name",
	                          const QString &value = QString(),
	                          const QString& description = QString(),
	                          rviz::Property* parent = 0,
	                          const char *changed_slot = 0,
	                          QObject* receiver = 0)
	   : rviz::EditableEnumProperty(name, value, description, parent, changed_slot, receiver)
	{}

public Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
};

/** Listbox showing all joint names of a RobotModel */
class JointNameProperty : public rviz::EditableEnumProperty {
	Q_OBJECT
public:
	explicit JointNameProperty(const QString &name = "Joint name",
	                           const QString &value = QString(),
	                           const QString &description = QString(),
	                           rviz::Property* parent = 0,
	                           const char *changed_slot = 0,
	                           QObject* receiver = 0)
	   : rviz::EditableEnumProperty(name, value, description, parent, changed_slot, receiver)
	{}

public Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
};

} // namespace rviz_cbf_plugin
