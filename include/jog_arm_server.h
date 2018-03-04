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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

// For jogging calc thread
void *joggingPipeline(void *threadid);

// For collision checking thread
void *collisionCheck(void *threadid);

// Shared variables
geometry_msgs::TwistStamped cmd_deltas;
pthread_mutex_t cmd_deltas_mutex;

sensor_msgs::JointState joints;
pthread_mutex_t joints_mutex;

trajectory_msgs::JointTrajectory new_traj;
pthread_mutex_t new_traj_mutex;

bool imminent_collision;
pthread_mutex_t imminent_collision_mutex;

// ROS subscriber callbacks
void delta_cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg);
void joints_cb(const sensor_msgs::JointStateConstPtr& msg);

// ROS params to be read
int readParams(ros::NodeHandle& n);
std::string move_group_name, joint_topic, cmd_in_topic, input_frame, cmd_out_topic, planning_frame;
double linear_scale, rot_scale, singularity_threshold, hard_stop_sing_thresh, low_pass_filter_coeff, pub_period, incoming_cmd_timeout;
bool simu;

std::string getStringParam(std::string s, ros::NodeHandle& n);
double getDoubleParam(std::string name, ros::NodeHandle& n);
bool getBoolParam(std::string name, ros::NodeHandle& n);
/**
 * Class lpf - Filter the joint velocities to avoid jerky motion.
 */
class lpf
{
  public:
    lpf(double low_pass_filter_coeff);
    double filter(const double& new_msrmt);
    double c_ = 10.;

  private:
    double prev_msrmts_ [3] = {0., 0., 0.};
    double prev_filtered_msrmts_ [2] = {0., 0.};
};

lpf::lpf(double low_pass_filter_coeff)
{
  c_ = low_pass_filter_coeff;
}

double lpf::filter(const double& new_msrmt)
{
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt = (1/(1+c_*c_+1.414*c_))*(prev_msrmts_[2]+2*prev_msrmts_[1]+prev_msrmts_[0]-(c_*c_-1.414*c_)*prev_filtered_msrmts_[1]-(-2*c_*c_+2)*prev_filtered_msrmts_[0]);;

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;
 
  return new_filtered_msrmt;
}

 
/**
 * Class JogCalcs - Provides the jog_arm action.
 */
class JogCalcs
{
public:
  JogCalcs(std::string move_group_name);
  
protected:
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
  
  double checkConditionNumber(const Eigen::MatrixXd &matrix) const;
  
  const robot_state::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr kinematic_state_;
  
  sensor_msgs::JointState jt_state_, orig_jts_;
  
  tf::TransformListener listener_;

  ros::Time prev_time_;

  double delta_t_;

  std::vector<jog_arm::lpf> filters_;

  // Check whether incoming cmds are stale. Pause if so
  ros::Duration time_of_incoming_cmd_;
};

class CollisionCheck
{
public:
    CollisionCheck(std::string move_group_name);
};

} // namespace jog_arm

#endif // JOG_ARM_SERVER_H
