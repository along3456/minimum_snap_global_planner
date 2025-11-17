#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "waypoint_tool.h"

namespace rviz
{

WaypointTool::WaypointTool()
  : Tool()
  , delete_tolerance_(0.5)
  , label_height_(0.6)
{
  shortcut_key_ = 'w';
  
  topic_property_ = new StringProperty("Topic", "/waypoints",
                                       "The topic on which to publish waypoints.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);

  feedback_topic_property_ = new StringProperty("Feedback Topic", "/waypoints/feedback",
                                                "Topic used to receive synchronized waypoint updates.",
                                                getPropertyContainer(), SLOT(updateFeedbackTopic()), this);
}

WaypointTool::~WaypointTool()
{
  clearWaypoints(false);
}

void WaypointTool::onInitialize()
{
  setName("Waypoint");
  updateTopic();
  updateFeedbackTopic();
}

void WaypointTool::activate()
{
  setStatus("Left click: Add waypoint. Right click near a marker: delete it. Right click elsewhere: clear all.");
}

void WaypointTool::deactivate()
{
}

void WaypointTool::updateTopic()
{
  waypoint_pub_ = nh_.advertise<geometry_msgs::PoseArray>(topic_property_->getStdString(), 1);
}

int WaypointTool::processMouseEvent(ViewportMouseEvent& event)
{
  int flags = 0;

  if (event.leftDown())
  {
    // Add waypoint on left click
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
    {
      addWaypoint(intersection);
      flags |= Render;
    }
  }
  else if (event.rightDown())
  {
    bool modified = false;
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
    {
      if (removeWaypointNear(intersection, delete_tolerance_))
      {
        modified = true;
      }
      else if (!waypoints_.empty())
      {
        clearWaypoints();
        modified = true;
      }
    }
    else if (!waypoints_.empty())
    {
      clearWaypoints();
      modified = true;
    }

    if (modified)
    {
      flags |= Render;
    }
  }

  return flags;
}

void WaypointTool::addWaypoint(const Ogre::Vector3& position)
{
  createWaypointVisual(position);
  publishWaypoints();

  ROS_INFO("Waypoint added at (%.2f, %.2f, %.2f). Total waypoints: %lu", 
           position.x, position.y, position.z, waypoints_.size());
}

void WaypointTool::createWaypointVisual(const Ogre::Vector3& position)
{
  waypoints_.push_back(position);

  Shape* marker = new Shape(Shape::Sphere, scene_manager_);
  marker->setColor(0.0f, 1.0f, 0.0f, 0.8f);  // Green color
  marker->setScale(Ogre::Vector3(0.3f, 0.3f, 0.3f));  // 0.3m diameter
  marker->setPosition(position);
  waypoint_markers_.push_back(marker);

  Ogre::SceneNode* text_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  text_node->setPosition(position.x, position.y, position.z + label_height_);
  MovableText* text = new MovableText("", "Liberation Sans", 0.25);
  text->setColor(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
  text->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
  text_node->attachObject(text);
  waypoint_label_nodes_.push_back(text_node);
  waypoint_labels_.push_back(text);

  updateWaypointLabels();
}

void WaypointTool::clearWaypoints(bool publish)
{
  // Clear visual markers
  for (size_t i = 0; i < waypoint_markers_.size(); ++i)
  {
    delete waypoint_markers_[i];
  }
  waypoint_markers_.clear();

  for (size_t i = 0; i < waypoint_labels_.size(); ++i)
  {
    if (waypoint_label_nodes_[i] && waypoint_labels_[i])
    {
      waypoint_label_nodes_[i]->detachObject(waypoint_labels_[i]);
    }
    delete waypoint_labels_[i];
    waypoint_labels_[i] = nullptr;

    if (waypoint_label_nodes_[i])
    {
      scene_manager_->destroySceneNode(waypoint_label_nodes_[i]);
    }
  }
  waypoint_labels_.clear();
  waypoint_label_nodes_.clear();

  // Clear waypoint data
  waypoints_.clear();

  // Publish empty waypoint array
  if (publish)
  {
    publishWaypoints();
    ROS_INFO("All waypoints cleared");
  }
}

void WaypointTool::removeWaypoint(size_t index)
{
  if (index >= waypoints_.size())
  {
    return;
  }

  delete waypoint_markers_[index];
  waypoint_markers_.erase(waypoint_markers_.begin() + index);

  if (waypoint_label_nodes_[index] && waypoint_labels_[index])
  {
    waypoint_label_nodes_[index]->detachObject(waypoint_labels_[index]);
  }
  delete waypoint_labels_[index];
  if (waypoint_label_nodes_[index])
  {
    scene_manager_->destroySceneNode(waypoint_label_nodes_[index]);
  }
  waypoint_labels_.erase(waypoint_labels_.begin() + index);
  waypoint_label_nodes_.erase(waypoint_label_nodes_.begin() + index);

  waypoints_.erase(waypoints_.begin() + index);

  updateWaypointLabels();
  publishWaypoints();

  ROS_INFO("Waypoint %lu removed. Remaining: %lu", index + 1, waypoints_.size());
}

bool WaypointTool::removeWaypointNear(const Ogre::Vector3& position, double tolerance)
{
  if (waypoints_.empty())
  {
    return false;
  }

  size_t closest_index = waypoints_.size();
  double closest_dist = tolerance;

  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    double dist = (waypoints_[i] - position).length();
    if (dist <= closest_dist)
    {
      closest_dist = dist;
      closest_index = i;
    }
  }

  if (closest_index < waypoints_.size())
  {
    removeWaypoint(closest_index);
    return true;
  }

  return false;
}

void WaypointTool::updateWaypointLabels()
{
  for (size_t i = 0; i < waypoint_labels_.size(); ++i)
  {
    if (waypoint_labels_[i])
    {
      waypoint_labels_[i]->setCaption(std::to_string(i + 1));
    }

    if (waypoint_label_nodes_[i])
    {
      const Ogre::Vector3& pos = waypoints_[i];
      waypoint_label_nodes_[i]->setPosition(pos.x, pos.y, pos.z + label_height_);
    }
  }
}

void WaypointTool::rebuildWaypoints(const geometry_msgs::PoseArray& msg)
{
  clearWaypoints(false);

  for (const auto& pose : msg.poses)
  {
    createWaypointVisual(poseToVector3(pose));
  }

  if (!msg.poses.empty())
  {
    ROS_INFO("Waypoint tool synchronized with %lu waypoint(s).", msg.poses.size());
  }

  context_->queueRender();
}

void WaypointTool::publishWaypoints()
{
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = context_->getFixedFrame().toStdString();
  pose_array.header.stamp = ros::Time::now();

  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    geometry_msgs::Pose pose;
    pose.position.x = waypoints_[i].x;
    pose.position.y = waypoints_[i].y;
    pose.position.z = waypoints_[i].z;
    pose.orientation.w = 1.0;  // No orientation
    pose_array.poses.push_back(pose);
  }

  waypoint_pub_.publish(pose_array);
}

void WaypointTool::updateFeedbackTopic()
{
  std::string topic = feedback_topic_property_->getStdString();

  waypoint_feedback_sub_.shutdown();

  if (topic.empty())
  {
    return;
  }

  waypoint_feedback_sub_ = nh_.subscribe(topic, 1, &WaypointTool::feedbackCallback, this);
}

void WaypointTool::feedbackCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if (!context_)
  {
    return;
  }

  rebuildWaypoints(*msg);
}

Ogre::Vector3 WaypointTool::poseToVector3(const geometry_msgs::Pose& pose) const
{
  return Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)

