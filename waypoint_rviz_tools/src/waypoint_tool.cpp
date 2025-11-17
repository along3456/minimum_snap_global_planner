/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "waypoint_tool.h"

namespace rviz
{

WaypointTool::WaypointTool()
  : Tool()
{
  shortcut_key_ = 'w';
  
  topic_property_ = new StringProperty("Topic", "/waypoints",
                                       "The topic on which to publish waypoints.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

WaypointTool::~WaypointTool()
{
  clearWaypoints();
}

void WaypointTool::onInitialize()
{
  setName("Waypoint");
  updateTopic();
}

void WaypointTool::activate()
{
  setStatus("Left click: Add waypoint. Right click: Clear all waypoints.");
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
    // Clear all waypoints on right click
    clearWaypoints();
    flags |= Render;
  }

  return flags;
}

void WaypointTool::addWaypoint(const Ogre::Vector3& position)
{
  // Store waypoint
  waypoints_.push_back(position);
  
  // Create visual marker (sphere)
  Shape* marker = new Shape(Shape::Sphere, scene_manager_);
  marker->setColor(0.0f, 1.0f, 0.0f, 0.8f);  // Green color
  marker->setScale(Ogre::Vector3(0.3f, 0.3f, 0.3f));  // 0.3m diameter
  marker->setPosition(position);
  waypoint_markers_.push_back(marker);
  
  // Publish waypoints
  publishWaypoints();
  
  ROS_INFO("Waypoint added at (%.2f, %.2f, %.2f). Total waypoints: %lu", 
           position.x, position.y, position.z, waypoints_.size());
}

void WaypointTool::clearWaypoints()
{
  // Clear visual markers
  for (size_t i = 0; i < waypoint_markers_.size(); ++i)
  {
    delete waypoint_markers_[i];
  }
  waypoint_markers_.clear();
  
  // Clear waypoint data
  waypoints_.clear();
  
  // Publish empty waypoint array
  publishWaypoints();
  
  ROS_INFO("All waypoints cleared");
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

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)

