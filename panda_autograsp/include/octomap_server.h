#pragma once

#include <ros/ros.h>

#include "panda_autograsp/ResetOctomap.h"

class OctomapServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer reset_ocotomap_srv_;
    std::string planning_group_;

public:
    OctomapServer();
    // ~OctomapServer()
    bool ResetOctomapService(panda_autograsp::ResetOctomap::Request &req,
                             panda_autograsp::ResetOctomap::Response &res);
};
