#ifndef CBF_DISPLAY_H
#define CBF_DISPLAY_H


#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#endif

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Pose.h>

#include <ros/publisher.h>

#include "rviz/selection/forwards.h"
#include "rviz/ogre_helpers/axes.h"

#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include "rviz/properties/status_property.h"


#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

class QMenu;

namespace rviz_cbf_plugin
{
class DisplayContext;

class CBFDisplay: public rviz::MessageFilterDisplay< ??? >  // <---!
{
Q_OBJECT
public:
  CBFDisplay();
  virtual ~CBFDisplay;

  void update(float wall_dt);

  //void setController( ??? );

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateActiveControllers;
  void updateListedControllers;

private:
  void processMessage( const ???_msgs::???::ConstPtr& msg );

  //boost::circular_buffer<?>;
}






}


#endif /* CBF_DISPLAY_H */
