#pragma once

#include <rviz/display.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <kdl/tree.hpp>

#include <boost/thread/mutex.hpp>

namespace rviz
{
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

private Q_SLOTS:
	void changedRobotDescription();

protected:
	void onInitialize();
	void onEnable();
	void onDisable();
	void fixedFrameChanged();

	void loadRobotModel();
	void onRobotModelLoaded();

protected:
	ros::NodeHandle node_handle_;

	// properties to show on side panel
	rviz::StringProperty *robot_description_property_;
	RobotDisplay *robot_display_;
	rviz::Property *controller_root_;

	// robot model
	boost::mutex robot_model_loading_mutex_;
	robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
	KDL::Tree kdl_tree_;

	// show our interactive markers
	rviz::Display *imarker_display_;

	// the config panel
	ConfigPanel *config_panel_;
	rviz::PanelDockWidget *config_panel_dock_;
};

} // namespace rviz_cbf_plugin
