#pragma once

#include <ros/ros.h>

#include "panda_autograsp/ResetOctomap.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class OctomapServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer reset_ocotomap_srv_;
    const std::string planning_group_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;

public:
    OctomapServer();
    // ~OctomapServer()
    bool ResetOctomapService(panda_autograsp::ResetOctomap::Request &req,
                             panda_autograsp::ResetOctomap::Response &res);
};
