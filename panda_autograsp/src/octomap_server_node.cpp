/** This server is used to control the moveit octomap. It contains the following
 * services:
 *  - reset_octomap
 */

#include "octomap_server.h"

int main(int argc, char **argv)
{

  // DEBUG
  int i = 0;
  while (i == 0)
  {
    usleep(100000); // sleep for 0.1 seconds
  }
  //DEBUG

  // Initialize node
  ros::init(argc, argv, "octomap_server");

  // Create octomap server
  OctomapServer ros_octomap_server;

  // Spin till shutdown
  ros::spin();
  return 0;
}
