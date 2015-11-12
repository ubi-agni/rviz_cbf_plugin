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

Panel::Panel(QWidget* parent, std::string tip_frame)
   : rviz::Panel(parent),
     rdf("robot_description"),
     server("cbf_marker_server")
{
	init("ee_link");
}

void Panel::load(const rviz::Config &config)
{

}

void Panel::save(rviz::Config config) const
{

}

void Panel::init(const std::string &tip_frame)
{
	srdf = rdf.getSRDF();
	if (!srdf) srdf.reset(new srdf::Model());
	urdf = rdf.getURDF();
	if (!urdf) {
		ROS_ERROR("couldn't load URDF model");
	}
	robot_model::RobotModelPtr robot_model(new robot_model::RobotModel(urdf, srdf));

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

	for (unsigned int i=0; i < kdl_chain.getNrOfSegments(); ++i) {
		KDL::Segment segment = kdl_chain.getSegment(i);
		links.push_back(segment.getName());
		KDL::Joint joint = segment.getJoint();
		joints.push_back(joint.getName());
	}

	server.clear();
	createJointMarkers();
	jsp = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void Panel::createJointMarkers()
{
  for (unsigned int i = 0; i < joints.size() && i < links.size(); i++)
  {
    createJointMarker(joints[i], links[i], i);
  }
}

void Panel::createJointMarker(const std::string joint_name, const std::string link_name, unsigned int segment_nr)
{
	geometry_msgs::PoseStamped stamped;
	stamped.header.frame_id = link_name;

	visualization_msgs::InteractiveMarker imarker = createInteractiveMarker(joint_name, stamped);
	double scale = imarker.scale = 0.2;

	std::string type = kdl_chain.getSegment(segment_nr).getJoint().getTypeName();

	

	// HERE you should use an orientation control (revolute) or position control (prismatic joint)

	
	if (type == "RotX") addOrientationControls(imarker, 4);
	if (type == "RotY") addOrientationControls(imarker, 2);
	if (type == "RotZ") addOrientationControls(imarker, 1);
	if (type == "TransX") addPositionControls(imarker, 4);
	if (type == "TransY") addPositionControls(imarker, 2);
	if (type == "TransZ") addPositionControls(imarker, 1);

	server.insert(imarker, boost::bind(&Panel::processFeedback, this, _1));
	server.applyChanges();
}

void Panel::processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
	marker_feedback = *feedback;
//	std::cout << marker_feedback << std::endl;
}

} // end namespace rviz_cbf_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cbf_plugin::Panel,rviz::Panel)
