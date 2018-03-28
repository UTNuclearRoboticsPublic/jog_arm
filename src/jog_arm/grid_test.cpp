///////////////////////////////////////////////////////////////////////////////
//      Title     : grid_test.cpp
//      Project   : jog_arm
//      Created   : 3/28/2018
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

// Test with motions to a grid of poses.

#include "jog_arm/grid_test.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jog_api_example");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	//////////////////////////////////////////////////
  // Send motion commands with this jog_api object.
  //////////////////////////////////////////////////
  std::string move_group_name = "manipulator";
  jog_api jogger(move_group_name);


  /////////////////////////////////////////
  // A start pose with good manipulabililty
  /////////////////////////////////////////
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = "world";
  start_pose.header.stamp = ros::Time::now();
  start_pose.pose.position.x = 0.4;
  start_pose.pose.position.y = -0.4;
  start_pose.pose.position.z = 0.6;
  start_pose.pose.orientation.x = 0.025;
  start_pose.pose.orientation.y = 0.247;
  start_pose.pose.orientation.z = 0.283;
  start_pose.pose.orientation.w = 0.926;


  /////////////////////////
  // Set up a grid of poses
  /////////////////////////
  std::vector<geometry_msgs::PoseStamped> poses;
  geometry_msgs::PoseStamped p = start_pose;

  double step = 0.1;
  for (double delta_x=-0.1; delta_x<=0.1; delta_x+=step)
  {
    for (double delta_y=-0.1; delta_y<=0.1; delta_y+=step)
    {
      for (double delta_z=-0.1; delta_z<=0.1; delta_z+=step)
      {
        p.pose.position.x += delta_x;
        p.pose.position.y += delta_y;
        p.pose.position.z += delta_z;
        poses.push_back(p);

        // Reset to start_pose for next iteration
        p.pose = start_pose.pose;
      }
    }
  }
  ROS_INFO_STREAM("Checking a grid of " << poses.size() <<" points.");

  ////////////////////////////////////////
  // move to every pose then back to start
  ////////////////////////////////////////
  int successes = 0;
  for(auto it = poses.begin(); it != poses.end(); ++it)
  {
    if (ros::ok())
    {
      // Move to the start pose
      ROS_INFO_STREAM("Moving to start pose");
      move_to_pose( jogger, start_pose );

      // Move to next grid pose
      ROS_INFO_STREAM("Moving to pose " << std::distance( poses.begin(),it ));
      if ( move_to_pose( jogger, *it ) )
        successes++;
      else
        ROS_WARN_STREAM("Failed to reach this pose.");
    }
    else
      return 1;
  }

  ROS_INFO_STREAM("Completed " << successes << " of " << poses.size() << " poses.");

  return 0;
}

bool move_to_pose(jog_api& jogger, geometry_msgs::PoseStamped& target_pose)
{
  // 1cm tolerance on the linear motion.
  // 0.01rad tolerance on the angular
  // Scale linear velocity commands between -0.5:0.5
  // Scale angular velocity commands between -1.0 : 1.0
  if ( !jogger.jacobian_move(target_pose, 0.01, 0.005, 0.3, 0.5))
  {
    ROS_ERROR_STREAM("Jacobian move failed");
    return 1;
  }

  return true;
}