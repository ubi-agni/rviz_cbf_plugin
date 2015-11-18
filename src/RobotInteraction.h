#pragma once

#include "interaction_types.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <boost/function.hpp>

namespace interactive_markers {
class InteractiveMarkerServer;
}

namespace rviz_cbf_plugin
{

/** Manage interactive markers for CBF controllers on a robot model.
 *
 * Markers corresponding to individual controllers can be enabled/disabled.
 * Different controllers can share the same marker, and share or add different
 * controls.
 * A controller registers one or more InteractionHandler objects each of
 * which maintains functions to (re)create interactive markers.
 */
class RobotInteraction
{
public:
	typedef boost::function<void(const int&)> GenericCallBackFn;
	typedef unsigned int CallbackID;
	enum AXES {X = 1, Y = 2, Z = 4, ALL = X | Y | Z};
public:
	/// The topic name on which the internal InteractiveMarkerServer operates
	static const std::string INTERACTIVE_MARKER_TOPIC;

	RobotInteraction(const std::string &ns = "");
	void setRobotState(const moveit::core::RobotStateConstPtr &rs);

	const std::string& getServerTopic(void) const {return topic_;}

	void clearMarkerList();
	void addMarkers(const std::list<LinkMarker> &markers);
	void addMarkers(const std::list<JointMarker> &markers);
	void addMarkers(const std::list<GenericMarker> &markers);
	void publishMarkers();
	void updateMarkerPoses();

	static double computeLinkMarkerSize(const std::string &link,
	                                    const moveit::core::RobotState &rs);

protected:
	struct MarkerDescription;
	typedef boost::shared_ptr<MarkerDescription> MarkerDescriptionPtr;

	MarkerDescriptionPtr getMarkerDescription(const std::string &name) const;
	MarkerDescriptionPtr getOrCreateMarkerDescription(const std::string &name);
	MarkerDescriptionPtr getUniqueMarkerDescription(const std::string &name);

	geometry_msgs::Pose getLinkPose(const std::string &link);

private:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void createLinkControls(MarkerDescriptionPtr &desc);
	void createJointControls(MarkerDescriptionPtr &desc);

private:
	moveit::core::RobotStateConstPtr robot_state_;
	interactive_markers::InteractiveMarkerServer *ims_;
	std::string topic_;

	// map marker name onto marker description
	typedef std::map<std::string, MarkerDescriptionPtr> MarkerMap;
	MarkerMap markers_;
};

} // namespace rviz_cbf_plugin
