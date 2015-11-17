#pragma once

#include <rviz/display.h>
#include <ros/ros.h>
#include <kdl/tree.hpp>

#include <boost/thread/mutex.hpp>

#include <moveit/macros/class_forward.h>
namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
}}
namespace robot_model_loader {
MOVEIT_CLASS_FORWARD(RobotModelLoader);
}
namespace rviz {
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class PanelDockWidget;
}

namespace rviz_cbf_plugin
{

class RobotDisplay;
class ConfigPanel;
class RootController;
MOVEIT_CLASS_FORWARD(RobotInteraction);
class ControllerDisplay : public rviz::Display
{
	Q_OBJECT

public:
	ControllerDisplay();
	~ControllerDisplay();

	void load(const rviz::Config& config);
	void save(rviz::Config config) const;

	void update(float wall_dt, float ros_dt);
	void reset();

public Q_SLOTS:
	void updateMarkers();

protected:
	void onInitialize();
	void onEnable();
	void onDisable();
	void fixedFrameChanged();

	void loadRobotModel();
	void onRobotModelLoaded();
	void initRobotState();

private Q_SLOTS:
	void changedRobotDescription();

protected:
	ros::NodeHandle node_handle_;

	// properties to show on side panel
	rviz::StringProperty *robot_description_property_;
	RobotDisplay   *robot_display_;
	RootController *controller_root_;

	// robot model
	boost::mutex robot_model_loading_mutex_;
	robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
	moveit::core::RobotStatePtr robot_state_;
	KDL::Tree kdl_tree_;

	// robot interaction: server + display
	RobotInteractionPtr robot_interaction_;
	rviz::Display *imarker_display_;

	// the config panel
	ConfigPanel *config_panel_;
};

} // namespace rviz_cbf_plugin
