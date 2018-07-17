///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_api.cpp
//      Project   : jog_arm
//      Created   : 3/27/2018
//      Author    : Andy Zelenak
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

// Provide a C++ interface for sending motion commands to the jog_arm server.

#include "jog_api/jog_api.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Publish cmds for a Cartesian motion to bring the robot to the target pose.
// Param linear_vel_scale: scales the velocity components of outgoing Twist msgs. Should be 0<linear_vel_scale<1
// Return true if successful.
//////////////////////////////////////////////////////////////////////////////////////////////////
bool jog_api::jacobianMove(geometry_msgs::PoseStamped& target_pose,
  const double trans_tolerance, 
  const double rot_tolerance,
  const double linear_vel_scale,
  const double rot_vel_scale,
  const ros::Duration& timeout)
{
  // Velocity scaling should be between 0 and 1
  if ( 0.>linear_vel_scale || 1.<linear_vel_scale )
  {
    ROS_ERROR_STREAM("[jog_api::jacobianMove] Velocity scaling parameter should be between 0 and 1.");
    return false;
  }
  if ( 0.>rot_vel_scale || 1.<rot_vel_scale )
  {
    ROS_ERROR_STREAM("[jog_api::jacobianMove] Velocity scaling parameter should be between 0 and 1.");
    return false;
  }

  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group_.getCurrentPose();
  // Transform to target frame
  transformPose(current_pose, target_pose.header.frame_id);

  // A structure to hold the result
  distanceAndTwist distanceAndTwist;
  distanceAndTwist = calculateDistanceAndTwist(current_pose, target_pose, linear_vel_scale, rot_vel_scale);

  ros::Time begin = ros::Time::now();

  // Is the current pose close enough?
  while (
    (distanceAndTwist.translational_distance > trans_tolerance || distanceAndTwist.rotational_distance > rot_tolerance)
    && ros::ok() )
  {
    // Have we timed out?
    if ( ros::Time::now()-begin > timeout )
      return false;

    // Get current robot pose
    current_pose = move_group_.getCurrentPose();

    transformPose(current_pose, target_pose.header.frame_id);

    // Update distance and twist to target
    distanceAndTwist = calculateDistanceAndTwist(current_pose, target_pose, linear_vel_scale, rot_vel_scale);

    // Publish the twist commands to move the robot
    jog_vel_pub_.publish(distanceAndTwist.twist);
    ros::Duration(0.01).sleep();
  }

  return true;
}


//////////////////////////////////////////////////////////////
// Maintain the current pose in given frame for given duration
//////////////////////////////////////////////////////////////
bool jog_api::maintainPose(std::string frame,
  ros::Duration duration,
  const double linear_vel_scale,
  const double rot_vel_scale)
{
  // Get initial pose -- to be maintained
  geometry_msgs::PoseStamped initial_pose;
  initial_pose = move_group_.getCurrentPose();
  transformPose(initial_pose, frame);

  ros::Time begin = ros::Time::now();
  distanceAndTwist distanceAndTwist;
  geometry_msgs::PoseStamped current_pose;

  while (
    ros::ok() &&
    (ros::Time::now() < (begin + duration))
  )
  {
    // Get current robot pose
    current_pose = move_group_.getCurrentPose();

    transformPose(current_pose, frame);
    ROS_ERROR_STREAM(current_pose);

    // Update distance and twist to target
    distanceAndTwist = calculateDistanceAndTwist(current_pose, initial_pose, linear_vel_scale, rot_vel_scale);

    // Publish the twist commands to move the robot
    jog_vel_pub_.publish(distanceAndTwist.twist);
    ros::Duration(0.01).sleep();    
  }

  return true;
}


////////////////////////////////////
// Transform a pose into given frame
////////////////////////////////////
bool jog_api::transformPose(geometry_msgs::PoseStamped &pose, std::string& desired_frame)
{
  // Remove a leading slash, if any
  if ( pose.header.frame_id.at(0) == '/' )
    pose.header.frame_id.erase(0,1);
  if ( desired_frame.at(0) == '/' )
    desired_frame.erase(0,1);

  try {
    geometry_msgs::TransformStamped current_frame_to_target = tf_buffer_.lookupTransform(pose.header.frame_id, desired_frame, ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(pose, pose, current_frame_to_target);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  } 

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Calculate Euclidean distance between 2 poses
// Returns a distanceAndTwist structure.
// distanceAndTwist.translational_distance holds the linear distance to target pose
// distanceAndTwist.rotational_distance holds the rotational distance to target pose
// distanceAndTwist.twist: these components have been normalized between -1:1,
// they can serve as motion commands as if from a joystick.
////////////////////////////////////////////////////////////////////////////////
jog_api::distanceAndTwist jog_api::calculateDistanceAndTwist(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::PoseStamped &target_pose,
  const double &linear_vel_scale, const double &rot_vel_scale)
{
  distanceAndTwist result;
  
  // Check frames on incoming PoseStampeds
  if ( current_pose.header.frame_id != target_pose.header.frame_id )
  {
    ROS_ERROR_STREAM("[arm_namespace::distanceAndTwist] Incoming PoseStampeds tf frames do not match.");
    return result;
  }
  result.twist.header.frame_id = current_pose.header.frame_id;
  result.twist.header.stamp = ros::Time::now();

  // Current pose: convert from quat to RPY
  tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double curr_r, curr_p, curr_y;
  m.getRPY(curr_r, curr_p, curr_y);

  // Target pose: convert from quat to RPY
  q = tf::Quaternion(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
  m = tf::Matrix3x3(q);
  double targ_r, targ_p, targ_y;
  m.getRPY(targ_r, targ_p, targ_y);

  /////////////////////////////////////
  // Calculate the twist to target_pose
  /////////////////////////////////////
  // Linear
  result.twist.twist.linear.x = target_pose.pose.position.x - current_pose.pose.position.x;
  result.twist.twist.linear.y = target_pose.pose.position.y - current_pose.pose.position.y;
  result.twist.twist.linear.z = target_pose.pose.position.z - current_pose.pose.position.z;

  // Angular
  result.twist.twist.angular.x = targ_r - curr_r;
  result.twist.twist.angular.y = targ_p - curr_p;
  result.twist.twist.angular.z = targ_y - curr_y;

  ///////////////////////////////////////////////////////////////////////////////
  // Normalize the twist to the target pose. Calculate distance while we're at it
  ///////////////////////////////////////////////////////////////////////////////
  // Linear:
  double sos = pow(result.twist.twist.linear.x, 2.); // Sum-of-squares
  sos += pow(result.twist.twist.linear.y, 2.);
  sos += pow(result.twist.twist.linear.z, 2.);
  result.translational_distance = pow( sos, 0.5 );

  result.twist.twist.linear.x = linear_vel_scale*result.twist.twist.linear.x/result.translational_distance;
  result.twist.twist.linear.y = linear_vel_scale*result.twist.twist.linear.y/result.translational_distance;
  result.twist.twist.linear.z = linear_vel_scale*result.twist.twist.linear.z/result.translational_distance;

  // Angular:
  sos = pow(result.twist.twist.angular.x, 2.);
  sos += pow(result.twist.twist.angular.y, 2.);
  sos += pow(result.twist.twist.angular.z, 2.);
  result.rotational_distance = pow( sos, 0.5 );

  // Ignore angle for now
  result.twist.twist.angular.x = rot_vel_scale*result.twist.twist.angular.x/result.rotational_distance;
  result.twist.twist.angular.y = rot_vel_scale*result.twist.twist.angular.y/result.rotational_distance;
  result.twist.twist.angular.z = rot_vel_scale*result.twist.twist.angular.z/result.rotational_distance;

  return result;
}
