#pragma once

#include <rviz/display.h>
#include <Eigen/Core>

namespace urdf
{
class ModelInterface;
}
namespace rviz
{
class Robot;
class BoolProperty;
class FloatProperty;
class DisplayContext;
}

namespace rviz_cbf_plugin
{

/** Visualize the robot
 *
 *  mimics moveit_rviz_plugin::RobotStateVisualization,
 *  adding some properties and updating links from joint states
 */
class RobotDisplay : public rviz::Display
{
	Q_OBJECT

public:
	RobotDisplay(rviz::Property *parent=0);

	void setModel(const urdf::ModelInterface &urdf);
	void reset();
	void update(const Eigen::VectorXd &joint_values);

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
	boost::shared_ptr<rviz::Robot> robot_;
};

} // namespace rviz_cbf_plugin
