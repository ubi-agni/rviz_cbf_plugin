#include "ControllerDisplay.h"
#include "RobotDisplay.h"
#include "ConfigPanel.h"

#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_factory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace rviz_cbf_plugin
{

ControllerDisplay::ControllerDisplay() :
   rviz::Display(),
   config_panel_dock_(NULL),
   imarker_display_(NULL)
{
	robot_description_property_ =
	      new rviz::StringProperty("Robot Description", "robot_description",
	                               "ROS parameter to load the URDF model from",
	                               this, SLOT(changedRobotDescription()), this);

	robot_display_ = new RobotDisplay();
	robot_display_->setName("Robot Visualization");
	this->addChild(robot_display_);

	controller_root_ = new rviz::Property("Controller", QVariant(), "", this);

	config_panel_ = new ConfigPanel();
}

ControllerDisplay::~ControllerDisplay()
{
	if (imarker_display_) delete imarker_display_;
	if (config_panel_dock_) delete config_panel_dock_;
}

void ControllerDisplay::onInitialize()
{
	Display::onInitialize();
	// initialize robot display
	robot_display_->initialize(context_);
	robot_display_->setEnabled(true);

	// initialize config panel
	rviz::WindowManagerInterface* window_context = context_->getWindowManager();
	if (window_context) {
		config_panel_dock_ = window_context->addPane("CBF Controller Config", config_panel_);
	}

	// display our markers
	imarker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
	imarker_display_->initialize(context_);

	// show children by default
	this->expand();

	loadRobotModel();
}

void ControllerDisplay::reset()
{
	Display::reset();
	robot_display_->reset();
	loadRobotModel();
}

void ControllerDisplay::changedRobotDescription()
{
	if (isEnabled())
		reset();
}

void ControllerDisplay::loadRobotModel()
{
	// wait for other robot loadRobotModel() calls to complete;
	boost::mutex::scoped_lock _(robot_model_loading_mutex_);

	robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description_property_->getStdString()));
	const boost::shared_ptr<urdf::ModelInterface> &urdf = robot_model_loader_->getURDF();
	if (!urdf) {
		setStatus(rviz::StatusProperty::Error, "robot model", "failed load URDF model");
		return;
	}

	if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree_)) {
		setStatus(rviz::StatusProperty::Error, "robot model", "failed to parse KDL tree");
		return;
	}

	onRobotModelLoaded();
}

void ControllerDisplay::onRobotModelLoaded()
{
	robot_display_->setModel(*robot_model_loader_->getURDF());

	// initialize robot state
	robot_state_.reset(new moveit::core::RobotState(robot_model_loader_->getModel()));
	robot_state_->update();
	robot_display_->update(boost::const_pointer_cast<const moveit::core::RobotState>(robot_state_));

	/* TODO
	robot_interaction_.reset(new robot_interaction::RobotInteraction(getRobotModel(), "rviz_moveit_motion_planning_display"));
	imarker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction_->getServerTopic() + "/update"));
	*/
}

void ControllerDisplay::onEnable()
{
	Display::onEnable();
	imarker_display_->setEnabled(true);
	imarker_display_->setFixedFrame(fixed_frame_);
}

void ControllerDisplay::onDisable()
{
	imarker_display_->setEnabled(false);
	Display::onDisable();
}

void ControllerDisplay::update(float wall_dt, float ros_dt)
{
	if (imarker_display_)
		imarker_display_->update(wall_dt, ros_dt);
	Display::update(wall_dt, ros_dt);

	// TODO perform controller step and update robot_display_
}

void ControllerDisplay::fixedFrameChanged()
{
	if (imarker_display_)
		imarker_display_->setFixedFrame(fixed_frame_);
}

void ControllerDisplay::load(const rviz::Config& config)
{
	Display::load(config);
}

void ControllerDisplay::save(rviz::Config config) const
{
	Display::save(config);
}

} // namespace rviz_cbf_plugin
