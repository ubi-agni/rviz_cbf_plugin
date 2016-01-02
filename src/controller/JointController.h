#pragma once
#include "Controller.h"
#include <Eigen/Core>
#include <cbf/primitive_controller.h>
#include <cbf/dummy_reference.h>
#include <cbf/dummy_resource.h>
#include <boost/thread/mutex.hpp>

namespace rviz {
}

namespace rviz_cbf_plugin
{

class JointNameProperty;
class JointController : public Controller
{
	Q_OBJECT
public:
	explicit JointController(const Controller &parent, const QString &name="Joint");
	CBF::PrimitiveControllerPtr getController() {return controller_;}
	void step(const moveit::core::RobotStatePtr &rs);

public Q_SLOTS:

protected:
	std::list<JointMarker> getJointMarkers() const;
	void computePrismaticTarget(const geometry_msgs::Pose &pose, const Eigen::Vector3d &axis, unsigned int joint_id) const;
	void computeRevoluteTarget(const geometry_msgs::Pose &pose, const Eigen::Vector3d &axis, unsigned int joint_id) const;
	void setTarget(unsigned int joint_id, double joint_pos);

protected Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
	void initController();

protected:
	boost::mutex controller_mutex_;
	CBF::PrimitiveControllerPtr controller_;
	CBF::DummyReferencePtr target_;
	CBF::DummyResourcePtr joint_values_;
	boost::shared_ptr<KDL::Chain> chain_;
	std::vector<KDL::Joint> joints_;
};

} // namespace rviz_cbf_plugin
