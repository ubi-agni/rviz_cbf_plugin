#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <std_msgs/Float64.h>
# include <geometry_msgs/Twist.h>
# include <geometry_msgs/Pose.h>
# include <rviz/panel.h>
# include <interactive_markers/interactive_marker_server.h>
# include <moveit/rdf_loader/rdf_loader.h>
# include <moveit/robot_model/robot_model.h>
# include <urdf_model/model.h>
# include <srdfdom/model.h>
# include <kdl_parser/kdl_parser.hpp>
# include <kdl/chain.hpp>
# include <kdl/tree.hpp>
# include <kdl/frames.hpp>
# include <kdl/chainfksolverpos_recursive.hpp>
# include <tf/tf.h>
# include <tf_conversions/tf_kdl.h>
# include <boost/foreach.hpp>
# include "marker_helpers.h"
#endif

namespace vm = visualization_msgs;
namespace im = interactive_markers;

namespace rviz_cbf_plugin {

class MarkerServer;

class Panel: public rviz::Panel
{
	Q_OBJECT

public:
	Panel(QWidget* parent = 0, std::string tip_frame = "base_link");

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;
	void init();
	void createJointPublishers();
	void createJointMarkers();
	void createJointMarker(const std::string joint_name, const std::string link_name);
	void processFeedback(const vm::InteractiveMarkerFeedbackConstPtr &feedback);

public Q_SLOTS:
	void setTopic(const QString& topic);

protected Q_SLOTS:
	void updateTopic();

protected:
	// The ROS node handle.
	ros::NodeHandle nh;
	interactive_markers::InteractiveMarkerServer server;
	vm::InteractiveMarkerFeedback marker_feedback;

	std::vector<std::string> joints, links;

	rdf_loader::RDFLoader rdf;
	boost::shared_ptr<srdf::Model> srdf;
	boost::shared_ptr<urdf::ModelInterface>& urdf;

	KDL::Tree kdl_tree;
	KDL::Chain kdl_chain;
	//KDL::ChainFkSolverPos_recursive fk;
	KDL::JntArray kdl_joints;
	KDL::Frame kdl_pose;
	tf::Pose tf_pose;
	geometry_msgs::PoseStamped stamped;

	std::map<std::string, ros::Publisher> joint_publishers;
};

} // end namespace rviz_cbf_plugin
