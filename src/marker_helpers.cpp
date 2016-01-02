/* ============================================================
 *
 * This file is a part of the RSB project
 *
 * Copyright (C) 2014 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the "LGPL"),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *     Bielefeld University
 *
 * ============================================================ */

#include "marker_helpers.h"
#include "interaction_types.h"
#include <QColor>

namespace marker_helpers {

void operator<< (geometry_msgs::Point &pos, const Eigen::Vector3d &p) {
	pos.x = p[0];
	pos.y = p[1];
	pos.z = p[2];
}

void operator<< (geometry_msgs::Quaternion &quat, const Eigen::Quaterniond &q)
{
	quat.w = q.w();
	quat.x = q.x();
	quat.y = q.y();
	quat.z = q.z();
}

geometry_msgs::Pose& operator<< (geometry_msgs::Pose &pose, const Eigen::Vector3d &p)
{
	pose.position << p;
	return pose;
}
geometry_msgs::Pose& operator<< (geometry_msgs::Pose &pose, const Eigen::Quaterniond &q)
{
	pose.orientation << q;
	return pose;
}

void operator<<(std_msgs::ColorRGBA& color, const QColor &c)
{
	color.r = c.redF();
	color.g = c.greenF();
	color.b = c.blueF();
	color.a = c.alphaF();
}

visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const Eigen::Vector3d &p)
{
	marker.pose.position << p;
	return marker;
}

visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const Eigen::Quaterniond &q)
{
	marker.pose.orientation << q;
	return marker;
}

visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const QColor &c)
{
	marker.color << c;
	return marker;
}

visualization_msgs::Marker &operator<<(visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &c)
{
	marker.color = c;
	return marker;
}


visualization_msgs::Marker
createBoxMarker(double x, double y, double z)
{
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = x;
	marker.scale.y = y;
	marker.scale.z = z;

	return marker;
}

visualization_msgs::Marker
createSphereMarker(double radius)
{
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.scale.x = radius * 2.0;
	marker.scale.y = radius * 2.0;
	marker.scale.z = radius * 2.0;

	return marker;
}

visualization_msgs::Marker
createArrowMarker(double scale,
                  const Eigen::Vector3d &dir)
{
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::ARROW;
	marker.scale.x = scale;
	marker.scale.y = 0.1*scale;
	marker.scale.z = 0.1*scale;

	marker.pose << Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir);

	return marker;
}


visualization_msgs::InteractiveMarker
createInteractiveMarker(const std::string &name,
                        const geometry_msgs::PoseStamped &stamped)
{
	visualization_msgs::InteractiveMarker imarker;
	imarker.name = name;
	imarker.header = stamped.header;
	imarker.pose = stamped.pose;
	return imarker;
}

visualization_msgs::InteractiveMarkerControl
createViewPlaneControl(bool position, bool orientation)
{
	visualization_msgs::InteractiveMarkerControl control;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;

	if (position && orientation)
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
   else if (orientation)
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
	else
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;

	control.independent_marker_orientation = true;
	control.always_visible = false;
	control.name = "move";

	return control;
}

void addPositionControl(visualization_msgs::InteractiveMarker& imarker,
                        const Eigen::Vector3d &axis, bool orientation_fixed) {
	visualization_msgs::InteractiveMarkerControl control;
	if (orientation_fixed)
		control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

	control.orientation << Eigen::Quaterniond::FromTwoVectors(
	                          Eigen::Vector3d::UnitX(), axis);
	imarker.controls.push_back(control);
}

void addOrientationControl(visualization_msgs::InteractiveMarker& imarker,
                           const Eigen::Vector3d &axis, bool orientation_fixed) {
	visualization_msgs::InteractiveMarkerControl control;
	if (orientation_fixed)
		control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

	control.orientation << Eigen::Quaterniond::FromTwoVectors(
	                          Eigen::Vector3d::UnitX(), axis);
	imarker.controls.push_back(control);
}

void addAxisControls(visualization_msgs::InteractiveMarker& imarker,
                 unsigned int axes, bool orientation_fixed)
{
	for (unsigned int i=0; i < 3; ++i) {
		if (!(axes & (rviz_cbf_plugin::TX << i))) continue;
		addPositionControl(imarker, Eigen::Vector3d::Unit(i), orientation_fixed);
	}
	for (unsigned int i=0; i < 3; ++i) {
		if (!(axes & (rviz_cbf_plugin::RX << i))) continue;
		addOrientationControl(imarker, Eigen::Vector3d::Unit(i), orientation_fixed);
	}
}

}
