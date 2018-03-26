#ifndef GET_ROS_PARAMS_H
#define GET_ROS_PARAMS_H

#include <ros/ros.h>
#include <string>

namespace get_ros_params
{
  std::string getStringParam(std::string s, ros::NodeHandle& n);
  double getDoubleParam(std::string name, ros::NodeHandle& n);
  double getIntParam(std::string name, ros::NodeHandle& n);
  bool getBoolParam(std::string name, ros::NodeHandle& n);
}

#endif // GET_ROS_PARAMS_H