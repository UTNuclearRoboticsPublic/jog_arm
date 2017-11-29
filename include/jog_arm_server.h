///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.h
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Blake Anderson, Andy Zelenak
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

/*
Server node for the arm jogging with MoveIt.
*/

#ifndef JOG_ARM_SERVER_H
#define JOG_ARM_SERVER_H

#include <Eigen/Eigenvalues>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace jog_arm {

// For a worker thread
void *joggingPipeline(void *threadid);

// Shared variables
geometry_msgs::TwistStamped cmd_deltas;
pthread_mutex_t cmd_deltas_mutex;
sensor_msgs::JointState joints;
pthread_mutex_t joints_mutex;

// ROS subscriber callbacks
void delta_cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg);
void joints_cb(const sensor_msgs::JointStateConstPtr& msg);

// ROS params to be read
void readParams(ros::NodeHandle& n);
std::string move_group_name, joint_topic, cmd_topic, moveit_planning_frame;
double linear_scale, rot_scale, singularity_threshold;

std::string getStringParam(std::string s, ros::NodeHandle& n);
double getDoubleParam(std::string name, ros::NodeHandle& n);
 
/**
 * Class JogArmServer - Provides the jog_arm action.
 */
class JogArmServer
{
public:
  /**
   * @brief: Default constructor for JogArmServer Class.
   */
  JogArmServer(std::string move_group_name);
  
protected:
  ros::NodeHandle nh_;

  ros::Publisher joint_trajectory_pub_;

  moveit::planning_interface::MoveGroupInterface arm_;

  geometry_msgs::TwistStamped cmd_deltas_;

  sensor_msgs::JointState incoming_jts_;
  
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  
  void jogCalcs(const geometry_msgs::TwistStamped& cmd);

  void updateJoints();

  Vector6d scaleCommand(const geometry_msgs::TwistStamped& command) const;
  
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &J) const;
  
  bool addJointIncrements(sensor_msgs::JointState &output, const Eigen::VectorXd &increments) const;

  bool updateJointVels(sensor_msgs::JointState &output, const Eigen::VectorXd &joint_vels) const;
  
  bool checkConditionNumber(const Eigen::MatrixXd &matrix) const;
  
  const robot_state::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr kinematic_state_;
  
  sensor_msgs::JointState jt_state_;
  
  tf::TransformListener listener_;

  ros::Time prev_time_;

  double delta_t_;
};

} // namespace jog_arm

#endif // JOG_ARM_SERVER_H
