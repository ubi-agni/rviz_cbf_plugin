#include "RobotDisplay.h"

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

using namespace moveit_rviz_plugin;

namespace rviz_cbf_plugin
{

RobotDisplay::RobotDisplay(rviz::Property *parent)
   : rviz::Display()
{
	if (!parent) parent = this;

	visual_enabled_property_ =
	      new rviz::BoolProperty("Show Robot Visual", true, "Indicates whether the robot state specified by the planning scene should be displayed as defined for visualisation purposes.",
	                             parent, SLOT(changedVisualEnabled()), this);

	collision_enabled_property_ =
	      new rviz::BoolProperty("Show Robot Collision", false, "Indicates whether the robot state specified by the planning scene should be displayed as defined for collision detection purposes.",
	                             parent, SLOT(changedCollisionEnabled()), this);

	alpha_property_ =
	      new rviz::FloatProperty( "Robot Alpha", 1.f, "Specifies the alpha for the robot links",
	                               parent, SLOT(changedAlpha()), this);
	alpha_property_->setMin( 0.0 );
	alpha_property_->setMax( 1.0 );
}

void RobotDisplay::onInitialize()
{
	Display::onInitialize();
	robot_.reset(new RobotStateVisualization(scene_node_, context_, "CBF", this));

	// set initial values
	changedVisualEnabled();
	changedCollisionEnabled();
	changedAlpha();

	robot_->setVisible(true);
}

void RobotDisplay::onEnable()
{
	Display::onEnable();
	calculateOffsetPosition();
}

void RobotDisplay::onDisable()
{
	Display::onDisable();
}

void RobotDisplay::setModel(const urdf::ModelInterface &urdf)
{
	robot_->load(urdf);
	calculateOffsetPosition();
}

void RobotDisplay::reset()
{
	if (!robot_) return;
	robot_->clear();
	robot_->setVisualVisible(visual_enabled_property_->getBool());
	robot_->setCollisionVisible(collision_enabled_property_->getBool());
}

void RobotDisplay::update(moveit::core::RobotStateConstPtr rs)
{
	robot_->update(rs);
}

void RobotDisplay::changedVisualEnabled()
{
	robot_->setVisualVisible(visual_enabled_property_->getBool());
}

void RobotDisplay::changedCollisionEnabled()
{
	robot_->setCollisionVisible(collision_enabled_property_->getBool());
}

void RobotDisplay::changedAlpha()
{
	robot_->setAlpha(alpha_property_->getFloat());
}

void RobotDisplay::setVisible(bool bVisible)
{
	robot_->setVisible(bVisible);
}

void RobotDisplay::fixedFrameChanged()
{
	Display::fixedFrameChanged();
	calculateOffsetPosition();
}

// calculate offset from fixed frame to robot root
void RobotDisplay::calculateOffsetPosition()
{
	if (!robot_ || !robot_->getRobot().getRootLink()) {
		setStatus(rviz::StatusProperty::Error, "robot model", "no root link");
		return;
	}
	const std::string &root_link = robot_->getRobot().getRootLink()->getName();
	tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), ros::Time(0), root_link);

	unsigned int attempts = 10;
	ros::Duration wait(0.1);
	while (attempts > 0 &&
	       !context_->getTFClient()->canTransform(fixed_frame_.toStdString(),
	                                              root_link, ros::Time(0))) {
		wait.sleep();
		--attempts;
	}

	try {
		if (attempts > 0) {
			context_->getTFClient()->transformPose(fixed_frame_.toStdString(), pose, pose);

			Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
			const tf::Quaternion &q = pose.getRotation();
			Ogre::Quaternion orientation(q.getW(), q.getX(), q.getY(), q.getZ());

			//planning_scene_node_->setPosition(position);
			//planning_scene_node_->setOrientation(orientation);
		} else {
			throw tf::TransformException("failed to transform from frame "
			                             + pose.frame_id_ + " to frame " + root_link);
		}
	} catch (tf::TransformException& e) {
		setStatus(rviz::StatusProperty::Error, "robot model", e.what());
	}
}

} // namespace rviz_cbf_plugin
