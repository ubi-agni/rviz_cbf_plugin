#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
# include <interactive_markers/interactive_marker_server.h>
# include <moveit/rdf_loader/rdf_loader.h>
# include <moveit/robot_model/robot_model.h>
# include <urdf_model/model.h>
# include <srdfdom/model.h>
# include <kdl/chain.hpp>
# include <kdl/tree.hpp>
#endif

namespace vm = visualization_msgs;

namespace rviz_cbf_plugin {

class MarkerServer;

class Panel: public rviz::Panel
{
	Q_OBJECT

public:
	Panel(QWidget* parent = 0, std::string tip_frame = "base_link");

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

public Q_SLOTS:
	void setTopic(const QString& topic);

protected Q_SLOTS:
	void updateTopic();

protected:
	void init(const std::string &tip_frame);
	void createJointMarkers();
	void createJointMarker(const std::string joint_name, const std::string link_name);
	void processFeedback(const vm::InteractiveMarkerFeedbackConstPtr &feedback);

private:
	// The ROS node handle.
	ros::NodeHandle nh;
	interactive_markers::InteractiveMarkerServer server;
	vm::InteractiveMarkerFeedback marker_feedback;

	std::vector<std::string> joints, links;

	rdf_loader::RDFLoader rdf;
	boost::shared_ptr<srdf::Model> srdf;
	boost::shared_ptr<urdf::ModelInterface> urdf;

	KDL::Tree kdl_tree;
	KDL::Chain kdl_chain;

	ros::Publisher jsp;
};

} // end namespace rviz_cbf_plugin
