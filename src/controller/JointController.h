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
	void markerCallback(const geometry_msgs::Pose &pose) const;

protected Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
	void initController();

protected:
	boost::mutex controller_mutex_;
	CBF::PrimitiveControllerPtr controller_;
	CBF::DummyReferencePtr target_;
	CBF::DummyResourcePtr joints_;
	boost::shared_ptr<KDL::Chain> chain_;
};

} // namespace rviz_cbf_plugin
