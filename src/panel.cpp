#include "panel.h"

namespace rviz_cbf_plugin {

Panel::Panel(QWidget* parent) : rviz::Panel(parent), rdf("robot_description")
{
public:


protected:
	srdf = rdf.getSRDF();
	if (!srdf) srdf.reset(new srdf::Model());
	urdf = rdf.getURDF();
	if (!urdf) {
		ROS_ERROR("couldn't load URDF model");
		return EXIT_FAILURE;
	}
	robot_model::RobotModelPtr robot_model(new robot_model::RobotModel(urdf, srdf));

	// fetch KDL tree
	if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree)) {
		ROS_ERROR("Could not initialize KDL tree");
		return EXIT_FAILURE;
	}

	if (!kdl_tree.getChain(kdl_tree.getRootSegment()->first, tip_frame, kdl_chain)) {
		ROS_ERROR_STREAM("Could not find chain to " << tip_frame);
		return EXIT_FAILURE;
	}
}

void Panel::load(const rviz::Config &config)
{

}

void Panel::save(rviz::Config config) const
{

}

void initJoints()
{
	createJointPublishers();
	createJointMarkers();
	//----!
}

void createJointPublishers()
{
  BOOST_FOREACH( std::string joint_name, joints )
  {
    joint_command_publishers[joint_name] = (nh.advertise<std_msgs::Float64>("/" + joint_name + "/command", 1, false));
  }

}

void createJointMarkers()
{
  for (unsigned int i = 0; i < joints.size() && i < links.size(); i++)
  {
    createJointMarker(joints[i], links[i]);
  }
}

void createJointMarker(const string joint_name, const string link_name)
  {
    	visualization_msgs::InteractiveMarker imarker = createInteractiveMarker("", stamped);
	imarker.name = joint_name;
	imarker.header.frame_id = link_name;
	double scale = imarker.scale = 0.2;

	visualization_msgs::InteractiveMarkerControl ctrl = createViewPlaneControl(true, true);
	visualization_msgs::Marker m = createSphereMarker(scale * 0.25);
	m << QColor(ok ? "lime" : "red");
	ctrl.markers.push_back(m);
	imarker.controls.push_back(ctrl);

	addPositionControls(imarker);
	addOrientationControls(imarker);

	server->clear();
	server->insert(imarker, &processFeedback);  // boost::bind ( , this, _1)
	server->applyChanges();
  }

void processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
	marker_feedback = *feedback;
//	std::cout << marker_feedback << std::endl;
}

} // end namespace rviz_cbf_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cbf_plugin::Panel,rviz::Panel)
