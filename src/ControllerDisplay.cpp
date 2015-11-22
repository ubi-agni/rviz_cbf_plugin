#include "ControllerDisplay.h"
#include "RobotDisplay.h"
#include "ConfigPanel.h"
#include "RobotInteraction.h"
#include "controller/Controller.h"
#include "controller/PositionController.h"
#include "controller/JointController.h"

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
   imarker_display_(NULL)
{
	robot_description_property_ =
	      new rviz::StringProperty("Robot Description", "robot_description",
	                               "ROS parameter to load the URDF model from",
	                               this, SLOT(changedRobotDescription()), this);

	robot_display_ = new RobotDisplay();
	robot_display_->setName("Robot Visualization");
	this->addChild(robot_display_);

	config_panel_ = new ConfigPanel();
	robot_interaction_.reset(new RobotInteraction());
	controller_root_ = new RootController(this, robot_interaction_);
	connect(controller_root_, SIGNAL(markersChanged()), this, SLOT(updateMarkers()));

	// TODO: for testing only:
	//new PositionController(*controller_root_);
	new JointController(*controller_root_);
}

ControllerDisplay::~ControllerDisplay()
{
	delete controller_root_;
	delete imarker_display_;
	robot_interaction_.reset();
}

void ControllerDisplay::onInitialize()
{
	Display::onInitialize();
	// initialize robot display
	robot_display_->initialize(context_);
	robot_display_->setEnabled(true);

	// associate config panel
	this->setAssociatedWidget(config_panel_);

	// display our interactive markers
	imarker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
	imarker_display_->initialize(context_);
	// listen to the marker topic of RobotInteraction
	imarker_display_->subProp("Update Topic")->setValue
	      (QString::fromStdString(robot_interaction_->getServerTopic() + "/update"));

	// show children by default
	this->expand();
	controller_root_->expand();

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
	auto robot_model = robot_model_loader_->getModel();
	// initialize robot state
	robot_state_.reset(new moveit::core::RobotState(robot_model));
	initRobotState();
	robot_display_->update(boost::const_pointer_cast<const moveit::core::RobotState>(robot_state_));

	// inform others about new model
	robot_interaction_->setRobotState(robot_state_);
	controller_root_->setRobotModel(robot_model);
}

void ControllerDisplay::updateMarkers()
{
	robot_interaction_->clearMarkerDescriptions();
	robot_interaction_->addMarkers(controller_root_->getLinkMarkers());
	robot_interaction_->addMarkers(controller_root_->getJointMarkers());
	robot_interaction_->addMarkers(controller_root_->getGenericMarkers());
	robot_interaction_->publishMarkers();
}

void ControllerDisplay::onEnable()
{
	Display::onEnable();
	robot_display_->setVisible(true);
	imarker_display_->setEnabled(true);
	imarker_display_->setFixedFrame(fixed_frame_);
}

void ControllerDisplay::onDisable()
{
	robot_display_->setVisible(false);
	imarker_display_->setEnabled(false);
	Display::onDisable();
}

void ControllerDisplay::update(float wall_dt, float ros_dt)
{
	if (imarker_display_)
		imarker_display_->update(wall_dt, ros_dt);
	Display::update(wall_dt, ros_dt);

	robot_interaction_->processCallbacks();
	controller_root_->step(robot_state_);
	robot_state_->update();
	robot_display_->update(robot_state_);
	robot_interaction_->updateMarkerPoses();
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

void ControllerDisplay::initRobotState()
{
	robot_state_->setToDefaultValues();
	// update pose of all links
	robot_state_->update();
}

} // namespace rviz_cbf_plugin
