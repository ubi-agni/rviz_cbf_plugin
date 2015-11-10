#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
# include <interactive_markers/interactive_marker_server.h>
# include <moveit/rdf_loader/rdf_loader.h>
# include <kdl/chain.hpp>
# include <kdl/tree.hpp>
# include <tf/tf.h>
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

public Q_SLOTS:
	void setTopic(const QString& topic);

protected Q_SLOTS:
	void updateTopic();

protected:
	// The ROS node handle.
	ros::NodeHandle nh;
	interactive_markers::InteractiveMarkerServer server;

	rdf_loader::RDFLoader rdf;
	boost::shared_ptr<srdf::Model> srdf;
	boost::shared_ptr<urdf::ModelInterface>& urdf;

	KDL::Tree kdl_tree;
	KDL::Chain kdl_chain;
};

} // end namespace rviz_cbf_plugin
