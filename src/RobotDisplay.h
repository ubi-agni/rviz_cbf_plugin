#pragma once

#include <rviz/display.h>

#include <moveit/macros/class_forward.h>
namespace moveit_rviz_plugin {
MOVEIT_CLASS_FORWARD(RobotStateVisualization);
}
namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
}}
namespace urdf {
class ModelInterface;
}
namespace rviz {
class Robot;
class BoolProperty;
class FloatProperty;
class DisplayContext;
}

namespace rviz_cbf_plugin
{

/** Visualize the robot
 *
 *  builds on moveit_rviz_plugin::RobotStateVisualization,
 *  adding some properties and updating links from joint states
 */
class RobotDisplay : public rviz::Display
{
	Q_OBJECT

public:
	RobotDisplay(rviz::Property *parent=0);

	void setModel(const urdf::ModelInterface &urdf);
	void reset();
	void update(moveit::core::RobotStateConstPtr rs);
	void setVisible(bool bVisible);

protected:
	void onInitialize();
	void onEnable();
	void onDisable();
	void fixedFrameChanged();
	void calculateOffsetPosition();

protected Q_SLOTS:
	void changedVisualEnabled();
	void changedCollisionEnabled();
	void changedAlpha();

protected:
	rviz::BoolProperty *visual_enabled_property_;
	rviz::BoolProperty *collision_enabled_property_;
	rviz::FloatProperty *alpha_property_;
	moveit_rviz_plugin::RobotStateVisualizationPtr robot_;
};

} // namespace rviz_cbf_plugin
