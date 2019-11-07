/** This server is used to control the moveit octomap. It contains the following
 * services:
 *  - reset_octomap
 */

#include "octomap_server.h"
#include "panda_autograsp/ResetOctomap.h"

#include <moveit/move_group_interface/move_group_interface.h>

OctomapServer::OctomapServer() : planning_group_("panda_arm")
{
  // Initialize octomap server services
  ROS_INFO("Initialising \'reset_octomap\' service...");
  reset_ocotomap_srv_ = nh_.advertiseService(
      "reset_octomap", &OctomapServer::ResetOctomapService, this);
  ROS_INFO("\'reset_octomap\' service initialized successfully.");

  // Create move group
  // moveit::planning_interface::MoveGroupInterface move_group_(planning_group_);
};

bool OctomapServer::ResetOctomapService(
    panda_autograsp::ResetOctomap::Request &req,
    panda_autograsp::ResetOctomap::Response &res)
{
  // ClearOctomap
  // move_group_.clearOctomap()
  ROS_INFO("Hello World");
  return true;
}