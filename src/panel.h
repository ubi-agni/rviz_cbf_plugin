#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
# include <interactive_markers/interactive_marker_server.h>
# include <sensor_msgs/JointState.h>
# include <moveit/rdf_loader/rdf_loader.h>
# include <moveit/robot_model/robot_model.h>
# include <urdf_model/model.h>
# include <srdfdom/model.h>
# include <kdl/chain.hpp>
# include <kdl/tree.hpp>

# include <cbf/dummy_reference.h>
# include <cbf/dummy_resource.h>
#endif

namespace vm = visualization_msgs;

namespace rviz_cbf_plugin {

class MarkerServer;

class Panel: public rviz::Panel
{
	Q_OBJECT

public:
	Panel(QWidget* parent = 0);

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

protected:
	void init(const std::string &tip_frame);
	void createJointMarkers();
	void createJointMarker(const KDL::Segment &segment);
	void createEEMarker(const geometry_msgs::PoseStamped &stamped, bool ok);
	void processFeedback(const vm::InteractiveMarkerFeedbackConstPtr &feedback);

private:
	// The ROS node handle
	ros::NodeHandle nh;
	interactive_markers::InteractiveMarkerServer server;
	vm::InteractiveMarkerFeedback marker_feedback;

	KDL::Chain kdl_chain;
	CBF::DummyResourcePtr  joints;
	CBF::DummyReferencePtr target;
	CBF::FloatVector target_vector;

	ros::Publisher jsp;
};

} // end namespace rviz_cbf_plugin
