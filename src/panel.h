#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

namespace rviz_cbf_plugin {

class Panel: public rviz::Panel
{
	Q_OBJECT

public:
	Panel(QWidget* parent = 0);

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

public slots:

protected:
	// The ROS node handle.
	ros::NodeHandle nh_;
};

} // end namespace rviz_cbf_plugin
