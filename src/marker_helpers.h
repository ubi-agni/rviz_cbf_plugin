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
#pragma once

#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

class QColor;

namespace marker_helpers {

void operator<< (geometry_msgs::Point &pos, const Eigen::Vector3d &p);
void operator<< (geometry_msgs::Quaternion &quat, const Eigen::Quaterniond &q);
geometry_msgs::Pose& operator<< (geometry_msgs::Pose &pose, const Eigen::Vector3d &p);
geometry_msgs::Pose& operator<< (geometry_msgs::Pose &pose, const Eigen::Quaterniond &q);

void operator<<(std_msgs::ColorRGBA &color, const QColor &c);
visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const Eigen::Vector3d &p);
visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const Eigen::Quaterniond &q);
visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const QColor &c);
visualization_msgs::Marker& operator<<(visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &c);

visualization_msgs::Marker createBoxMarker(double x, double y, double z);
visualization_msgs::Marker createSphereMarker(double radius);
visualization_msgs::Marker createArrowMarker(double scale, const Eigen::Vector3d &dir);

visualization_msgs::InteractiveMarker
createInteractiveMarker(const std::string &name,
                        const geometry_msgs::PoseStamped &stamped);

visualization_msgs::InteractiveMarkerControl
createViewPlaneControl(bool position, bool orientation);

void addPositionControl(visualization_msgs::InteractiveMarker& imarker,
                        const Eigen::Vector3d &axis, bool orientation_fixed = false);
void addOrientationControl(visualization_msgs::InteractiveMarker& imarker,
                           const Eigen::Vector3d &axis, bool orientation_fixed = false);
void addAxisControls(visualization_msgs::InteractiveMarker& imarker, unsigned int axes,
                 bool orientation_fixed = false);

}
