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

class LinkNameProperty;
class PositionController : public Controller
{
	Q_OBJECT
public:
	explicit PositionController(const Controller &parent, const QString &name="Position");
	CBF::PrimitiveControllerPtr getController() {return controller_;}
	void step(const moveit::core::RobotStatePtr &rs);

public Q_SLOTS:
	void setLink(const std::string &name);

protected:
	std::list<LinkMarker> getLinkMarkers() const;
	void markerCallback(const geometry_msgs::Pose &pose) const;
	void setTarget(const Eigen::Vector3d &position);

protected Q_SLOTS:
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);
	void changedLinkName();
	void initController();

protected:
	std::string link_name_;
	LinkNameProperty *link_name_property_;

	boost::mutex controller_mutex_;
	CBF::PrimitiveControllerPtr controller_;
	CBF::DummyReferencePtr target_;
	CBF::DummyResourcePtr joints_;
	boost::shared_ptr<KDL::Chain> chain_;
};

} // namespace rviz_cbf_plugin
