#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

namespace rviz_cbf_plugin {

class MarkerServer;

class Panel: public rviz::Panel
{
	Q_OBJECT

public:
	Panel(QWidget* parent = 0);

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;
	void initJoints();

public Q_slots:

	void setTopic(const QString& topic);

protected Q_slots:

	void updateTopic();

protected:
	// The ROS node handle.
	ros::NodeHandle nh;
	interactive_markers::InteractiveMarkerServer server;
	tf::TransformListener tf_listener;

	rdf_loader::RDFLoader rdf;
	boost::shared_ptr<srdf::Model> srdf;
	boost::shared_ptr<urdf::ModelInterface>& urdf;

	KDL::Tree kdl_tree;
	KDL::Chain kdl_chain;
	
	map<std::string, interactive_markers::MenuHandler> joint_menu_handlers;

	bool in_move;

	ros::Timer move_timer;
	double move_time;

	map<std::string, ros::Publisher> joint_publishers;
};

} // end namespace rviz_cbf_plugin
