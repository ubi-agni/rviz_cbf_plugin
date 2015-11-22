#include "properties.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/joint_model.h>
#include <boost/foreach.hpp>

namespace rviz_cbf_plugin
{

void LinkNameProperty::setRobotModel(const moveit::core::RobotModelConstPtr &rm)
{
	clearOptions();
	const auto links = rm->getLinkModels();
	BOOST_FOREACH(const robot_model::LinkModel *link, links) {
		addOptionStd(link->getName());
	}
	sortOptions();
}

void JointNameProperty::setRobotModel(const moveit::core::RobotModelConstPtr &rm)
{
	clearOptions();
	const auto joints = rm->getJointModels();
	BOOST_FOREACH(const robot_model::JointModel *joint, joints) {
		addOptionStd(joint->getName());
	}
	sortOptions();
}

} // namespace rviz_cbf_plugin
