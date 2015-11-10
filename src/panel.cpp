#include "panel.h"
#include <sensor_msgs/JointState.h>

namespace rviz_cbf_plugin {

Panel::Panel(QWidget* parent, std::string tip_frame)
   : rviz::Panel(parent),
     rdf("robot_description"),
     server("cbf_marker_server")
{
	init("tool_frame");
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

	// fetch KDL tree
	if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree)) {
		ROS_ERROR("Could not initialize KDL tree");
	}

	if (!kdl_tree.getChain(kdl_tree.getRootSegment()->first, tip_frame, kdl_chain)) {
		ROS_ERROR_STREAM("Could not find chain to " << tip_frame);
	}

	//fk = KDL::ChainFkSolverPos_recursive(kdl_chain);
	kdl_joints = KDL::JntArray(kdl_chain.getNrOfJoints());
	//fk.JntToCart(kdl_joints, kdl_pose);
	KDL::ChainFkSolverPos_recursive(kdl_chain).JntToCart(kdl_joints, kdl_pose);

	tf::poseKDLToTF(kdl_pose, tf_pose);

	stamped.header.frame_id = kdl_tree.getRootSegment()->first;
	tf::poseTFToMsg(tf_pose, stamped.pose);

	for (unsigned int i=0; i < kdl_chain.getNrOfSegments(); ++i) {
		KDL::Segment segment = kdl_chain.getSegment(i);
		links.push_back(segment.getName());
		KDL::Joint joint = segment.getJoint();
		joints.push_back(joint.getName());
	}

	createJointMarkers();
	jsp = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void Panel::createJointMarkers()
{
  for (unsigned int i = 0; i < joints.size() && i < links.size(); i++)
  {
    createJointMarker(joints[i], links[i]);
  }
}

void Panel::createJointMarker(const std::string joint_name, const std::string link_name)
{
	visualization_msgs::InteractiveMarker imarker = createInteractiveMarker("", stamped);
	imarker.name = joint_name;
	imarker.header.frame_id = link_name;
	double scale = imarker.scale = 0.2;

	// HERE you should use an orientation control (revolute) or position control (prismatic joint)
	addOrientationControls(imarker, 1);

	server.clear();
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
