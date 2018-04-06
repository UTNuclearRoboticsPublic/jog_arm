///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_api_example.cpp
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

// Perform a motion with the jog_arm API

#include "support/jog_api_example.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jog_api_example");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	//////////////////////////////////////////////////
  // Send motion commands with this jog_api object.
  // Put your robot name here -- often "manipulator"
  //////////////////////////////////////////////////
  std::string move_group_name = "manipulator";
  jog_api jogger(move_group_name);

  ////////////////////////////
  // Move to a good start pose
  ////////////////////////////
  geometry_msgs::PoseStamped new_pose;
  new_pose.header.frame_id = "world";
  new_pose.header.stamp = ros::Time::now();
  new_pose.pose.position.x = 0.4;
  new_pose.pose.position.y = 0.23;
  new_pose.pose.position.z = 0.4;
  new_pose.pose.orientation.x = 0.025;
  new_pose.pose.orientation.y = 0.247;
  new_pose.pose.orientation.z = 0.283;
  new_pose.pose.orientation.w = 0.926;

  // 1cm tolerance on the linear motion.
  // 0.01rad tolerance on the angular
  // Scale linear velocity commands between -0.5:0.5
  // Scale angular velocity commands between -1.0 : 1.0
  // Timeout, i.e. stop sending commands, after 10s
  if ( !jogger.jacobian_move(new_pose, 0.01, 0.01, 0.5, 1.0, ros::Duration(10)) )
	{
  	ROS_ERROR_STREAM("Jacobian move failed");
  	return 1;
	}

  ////////////////////////////////////////////
  // Get robot's current pose via MoveIt's api
  ////////////////////////////////////////////
  moveit::planning_interface::MoveGroupInterface mgi(move_group_name);
  geometry_msgs::PoseStamped current_pose = mgi.getCurrentPose();

  ROS_INFO_STREAM("Current pose: " << current_pose);

  return 0;
}