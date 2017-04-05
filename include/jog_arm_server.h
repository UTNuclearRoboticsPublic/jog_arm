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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf/transform_listener.h>

namespace jog_arm {
 
/**
 * Class JogArmServer - Provides the jog_arm action.
 */
class JogArmServer
{
public:
  /**
   * @breif: Default constructor for JogArmServer Class.
   */
  JogArmServer(std::string move_group_name, std::string cmd_topic_name);
  
protected:
  
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  
  void commandCB(geometry_msgs::TwistStampedConstPtr msg);

  void jointStateCB(sensor_msgs::JointStateConstPtr msg);

  float overallScaling;
  Vector6d scaleCommand(const geometry_msgs::TwistStamped& command, const Vector6d& scalar) const;
  
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &J) const;
  
  bool addJointIncrements(sensor_msgs::JointState &output, const Eigen::VectorXd &increments) const;
  
  bool checkConditionNumber(const Eigen::MatrixXd &matrix, double threshold) const;

  ros::NodeHandle nh_;
  
  ros::Subscriber joint_sub_, cmd_sub_;
  
  moveit::planning_interface::MoveGroupInterface arm_;
  const robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr kinematic_state_;
  
  sensor_msgs::JointState current_joints_;
  
  std::vector<std::string> joint_names_;
  
  ros::AsyncSpinner spinner_; // Motion planner requires an asynchronous spinner
  
  tf::TransformListener listener_;
};

} // namespace jog_arm

#endif // JOG_ARM_SERVER_H
