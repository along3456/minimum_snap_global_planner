#ifndef RVIZ_WAYPOINT_TOOL_H
#define RVIZ_WAYPOINT_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <OGRE/OgreVector3.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include "rviz/tool.h"
#endif
#include <vector>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Shape;
class DisplayContext;
class StringProperty;
class MovableText;

class WaypointTool: public Tool
{
Q_OBJECT
public:
  WaypointTool();
  virtual ~WaypointTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent(ViewportMouseEvent& event);

private Q_SLOTS:
  void updateTopic();
  void updateFeedbackTopic();

private:
  void addWaypoint(const Ogre::Vector3& position);
  void createWaypointVisual(const Ogre::Vector3& position);
  void clearWaypoints(bool publish = true);
  void removeWaypoint(size_t index);
  bool removeWaypointNear(const Ogre::Vector3& position, double tolerance);
  void updateWaypointLabels();
  void rebuildWaypoints(const geometry_msgs::PoseArray& msg);
  void publishWaypoints();
  void feedbackCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  Ogre::Vector3 poseToVector3(const geometry_msgs::Pose& pose) const;

  std::vector<Ogre::Vector3> waypoints_;
  std::vector<Shape*> waypoint_markers_;
  std::vector<MovableText*> waypoint_labels_;
  std::vector<Ogre::SceneNode*> waypoint_label_nodes_;
  double delete_tolerance_;
  double label_height_;
  
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Subscriber waypoint_feedback_sub_;
  
  StringProperty* topic_property_;
  StringProperty* feedback_topic_property_;
};

}

#endif // RVIZ_WAYPOINT_TOOL_H

