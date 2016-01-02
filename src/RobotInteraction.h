#pragma once

#include "interaction_types.h"

#include <visualization_msgs/InteractiveMarkerFeedback.h>
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
 *
 * Markers are internally collected in a map of MarkerDescriptions.
 * Only if they will be published to the InteractiveMarkerServer, we create
 * interactive markers from the descriptions.
 * This allows to summarizes several controls requested by different controllers
 * into a single interactive marker
 */
class RobotInteraction
{
public:
	/// The topic name on which the internal InteractiveMarkerServer operates
	static const std::string INTERACTIVE_MARKER_TOPIC;

	RobotInteraction(const std::string &ns = "");
	~RobotInteraction();
	void setRobotState(const moveit::core::RobotStateConstPtr &rs);

	const std::string& getServerTopic(void) const {return topic_;}

	/// clear list of marker descriptions
	void clearMarkerDescriptions();
	/// add marker descriptions
	void addMarkers(const std::list<LinkMarker> &markers);
	void addMarkers(const std::list<JointMarker> &markers);
	void addMarkers(const std::list<GenericMarker> &markers);
	/// create actual interactive markers from descriptions and publish them
	void publishMarkers();
	/// set marker poses from current robot state
	void updateMarkerPoses();
	/// process poseUpdates received so far
	void processCallbacks();

	double computeLinkMarkerSize(const std::string &target_link_name,
	                             std::string *actual_link_name = NULL);

protected:
	struct MarkerDescription;
	typedef boost::shared_ptr<MarkerDescription> MarkerDescriptionPtr;

	MarkerDescriptionPtr getMarkerDescription(const std::string &name) const;
	MarkerDescriptionPtr getOrCreateMarkerDescription(const std::string &name);
	MarkerDescriptionPtr getUniqueMarkerDescription(const std::string &name);

	/// get the current pose of the link w.r.t. root frame
	geometry_msgs::Pose getLinkPose(const std::string &link);

	/// add interactive control based on link geometry
	bool addLinkControl(const std::string &link, unsigned int interaction_mode,
	                    visualization_msgs::InteractiveMarker &im) const;
	bool addSphereControl(unsigned int interaction_mode,
	                      visualization_msgs::InteractiveMarker &im) const;

private:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
	moveit::core::RobotStateConstPtr robot_state_;
	interactive_markers::InteractiveMarkerServer *ims_;
	std::string topic_;

	// map marker name onto marker description
	typedef std::map<std::string, MarkerDescriptionPtr> MarkerMap;
	MarkerMap markers_;
};

} // namespace rviz_cbf_plugin
