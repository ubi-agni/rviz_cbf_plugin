#pragma once

#include <moveit/robot_model/robot_model.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
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
	void setRobotModel(const moveit::core::RobotModelConstPtr &rm);

	const std::string& getServerTopic(void) const {return topic_;}


	/** add a generic interactive control
	 * @param marker_name defines the marker, the control should be added too
	 * @param control_name can be one of view, tx,ty,tz, rx,ry,rz, or be user defined
	 * @param control
	 * @param callback
	 */
	CallbackID addGenericControl(const std::string& marker_name,
	                             const std::string& control_name,
	                             visualization_msgs::InteractiveMarkerControl &control,
	                             GenericCallBackFn callback);
	CallbackID addPositionControl(const std::string& marker_name,
	                              PositionCallbackFn callback,
	                              unsigned int axes = AXES::ALL);
	CallbackID addOrientationControl(const std::string& marker_name,
	                                 OrientationCallbackFn callback,
	                                 unsigned int axes = AXES::ALL);
private:
	/** Retrieve a unique ID for a specific controller callback
	 *
	 * The ID is used to enable, disable, and remove specific callbacks
	 */
	CallbackID getCallbackId();

private:
	robot_model::RobotModelConstPtr robot_model_;
	interactive_markers::InteractiveMarkerServer *ims_;
	std::string topic_;

	// map control name (view, tx,ty,tz, rx,ry,rz, ...) to interactive marker control
	typedef std::map<std::string, visualization_msgs::InteractiveMarkerControl> ControlsMap;
	typedef std::map<std::string, ControlsMap> markers_;

	CallbackID nextId_;
	std::set<CallbackID> active_callbacks_;
	std::set<CallbackID> inactive_callbacks_;
	std::map<CallbackID, GenericCallBackFn> callbacks_;
};

} // namespace rviz_cbf_plugin
