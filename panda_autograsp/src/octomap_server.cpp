/** This server is used to control the moveit octomap. It contains the following
 * services:
 *  - reset_ocotomap
*/

#include "ros/ros.h"
#include "panda_autograsp/ResetOctomap.h"

bool reset_octomap_service(panda_autograsp::ResetOctomap::Request  &req,
         panda_autograsp::ResetOctomap::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("reset_octomap", reset_octomap_service);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
