#include "panel.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/foreach.hpp>

#include "marker_helpers.h"

namespace im = interactive_markers;
using namespace marker_helpers;

namespace rviz_cbf_plugin {

Panel::Panel(QWidget* parent)
   : rviz::Panel(parent),
     server("cbf_marker_server")
{
	init("tool0");
}

void Panel::load(const rviz::Config &config)
{
	rviz::Panel::load(config);
}

void Panel::save(rviz::Config config) const
{
	rviz::Panel::save(config);
}

void Panel::init(const std::string &tip_frame)
{
	rdf_loader::RDFLoader rdf("robot_description");
	boost::shared_ptr<srdf::Model> srdf = rdf.getSRDF();
	if (!srdf) srdf.reset(new srdf::Model());
	boost::shared_ptr<urdf::ModelInterface> urdf = rdf.getURDF();
	if (!urdf) {
		ROS_ERROR("couldn't load URDF model");
	}
	robot_model::RobotModelPtr robot_model(new robot_model::RobotModel(urdf, srdf));

	KDL::Tree kdl_tree;
	if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree)) {
		ROS_ERROR("Could not initialize KDL tree");
	}

	if (!kdl_tree.getChain(kdl_tree.getRootSegment()->first, tip_frame, kdl_chain)) {
		ROS_ERROR_STREAM("Could not find chain to " << tip_frame);
	}

	auto fk = KDL::ChainFkSolverPos_recursive(kdl_chain);
	KDL::JntArray kdl_joints = KDL::JntArray(kdl_chain.getNrOfJoints());
	KDL::Frame kdl_pose;
	fk.JntToCart(kdl_joints, kdl_pose);

	tf::Pose tf_pose;
	tf::poseKDLToTF(kdl_pose, tf_pose);

	geometry_msgs::PoseStamped stamped;
	stamped.header.frame_id = kdl_tree.getRootSegment()->first;
	tf::poseTFToMsg(tf_pose, stamped.pose);

	server.clear();
	createEEMarker(stamped, true);
	createJointMarkers();
	server.applyChanges();

	jsp = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void Panel::createJointMarkers()
{
	for (unsigned int i=0; i < kdl_chain.getNrOfSegments(); ++i) {
		const KDL::Segment &segment = kdl_chain.getSegment(i);
		createJointMarker(segment);
	}
}

void Panel::createJointMarker(const KDL::Segment &segment)
{
	const std::string &link_name = segment.getName();
	geometry_msgs::PoseStamped stamped;
	stamped.header.frame_id = link_name;

	visualization_msgs::InteractiveMarker imarker = createInteractiveMarker(link_name, stamped);
	double scale = imarker.scale = 0.2;

	const KDL::Joint &joint = segment.getJoint();
	switch (joint.getType()) {
	case KDL::Joint::RotX: addOrientationControls(imarker, AXES::X); break;
	case KDL::Joint::RotY: addOrientationControls(imarker, AXES::Y); break;
	case KDL::Joint::RotZ: addOrientationControls(imarker, AXES::Z); break;

	case KDL::Joint::TransX: addPositionControls(imarker, AXES::X); break;
	case KDL::Joint::TransY: addPositionControls(imarker, AXES::Y); break;
	case KDL::Joint::TransZ: addPositionControls(imarker, AXES::Z); break;
	}
	addOrientationControls(imarker, AXES::X);

	std::cout << "add marker with " << imarker.controls.size() << " controls" << std::endl;
	server.insert(imarker, boost::bind(&Panel::processFeedback, this, _1));
}

void Panel::createEEMarker(const geometry_msgs::PoseStamped &stamped, bool ok)
{
	vm::InteractiveMarker imarker = createInteractiveMarker("EE", stamped);
	double scale = imarker.scale = 0.2;

	visualization_msgs::InteractiveMarkerControl ctrl = createViewPlaneControl(true, true);
	visualization_msgs::Marker m = createSphereMarker(scale * 0.25);
	m << QColor(ok ? "lime" : "red");
	ctrl.markers.push_back(m);
	imarker.controls.push_back(ctrl);

	addPositionControls(imarker);
	addOrientationControls(imarker);

	server.insert(imarker, boost::bind(&Panel::processFeedback, this, _1));
}

void Panel::processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
	marker_feedback = *feedback;
}

} // end namespace rviz_cbf_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cbf_plugin::Panel,rviz::Panel)
