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

  double step = 0.03;
  for (double delta_x=-0.03; delta_x<=0.06; delta_x+=step)
  {
    for (double delta_y=-0.03; delta_y<=0.06; delta_y+=step)
    {
      for (double delta_z=-0.03; delta_z<=0.06; delta_z+=step)
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


  /////////////////
  // Set up MoveIt!
  /////////////////
  moveit::planning_interface::MoveGroupInterface mgi(move_group_name);
  mgi.setPlannerId("RRTConnectkConfigDefault");
  mgi.setGoalPositionTolerance(0.01);
  mgi.setGoalOrientationTolerance(0.02);
  mgi.setPlanningTime(5);
  mgi.setMaxVelocityScalingFactor(0.2);
  mgi.setPoseReferenceFrame(start_pose.header.frame_id);
  std::vector<geometry_msgs::Pose> waypoints;

  // Move to start pose
  //mgi.setPoseTarget(start_pose);
  std::vector<double> start_joints = {-1.343, -1.485, 1.697, 2.042, 0.596, 2.572};
  mgi.setJointValueTarget(start_joints);
  if ( !mgi.move() )
  {
    ROS_ERROR_STREAM("Move to start pose failed. Exiting.");
    return 1;
  }


  ////////////////////////////////////////
  // Test the jog_arm Cartesian motion API
  ////////////////////////////////////////
  ros::Time begin = ros::Time::now();
  int jog_arm_successes = 0;
/*
  for(auto it = poses.begin(); it != poses.end(); ++it)
  {
    if (ros::ok())
    {
      // Move to next grid pose
      ROS_INFO_STREAM("Moving to pose " << std::distance( poses.begin(),it ));
      if ( move_to_pose( jogger, *it ) )
        jog_arm_successes++;
      else
        ROS_WARN_STREAM("Failed to reach this pose.");
    }
    else
      return 1;
  }

  ROS_INFO_STREAM("Completed " << jog_arm_successes << " of " << poses.size() << " poses.");
  ROS_INFO_STREAM("Trial took " << ros::Time::now()-begin << " seconds.");
*/
  // Move to start pose
  mgi.setJointValueTarget(start_joints);
  if ( !mgi.move() )
  {
    ROS_ERROR_STREAM("Move to start pose failed. Exiting.");
    return 1;
  }


  ////////////////////////////////////////////
  // Now test the MoveIt! Cartesian motion API
  ////////////////////////////////////////////
  begin = ros::Time::now();
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  int moveit_successes = 0;
  double fraction = 0.;

  for(auto it = poses.begin(); it != poses.end(); ++it)
  {
    if (ros::ok())
    {
      // Move to next grid pose
      ROS_INFO_STREAM("Moving to pose " << std::distance( poses.begin(),it ));
      waypoints.clear();
      waypoints.push_back( (*it).pose );
      fraction = mgi.computeCartesianPath(waypoints, 0.005, 15., trajectory);
      ROS_INFO_STREAM(fraction);
      if (fraction == 1.)
        moveit_successes++;
      else
        ROS_WARN_STREAM("Failed to reach this pose.");
      plan.trajectory_ = trajectory;
      mgi.execute(plan);
    }
    else
      return 1;
  }

  ROS_INFO_STREAM("MoveIt! completed " << moveit_successes << " of " << poses.size() << " poses.");
  ROS_INFO_STREAM("MoveIt! trial took " << ros::Time::now()-begin << " seconds.");

  return 0;
}

bool move_to_pose(jog_api& jogger, geometry_msgs::PoseStamped& target_pose)
{
  // 1cm tolerance on the linear motion.
  // 0.05rad tolerance on the angular
  // Scale linear velocity commands between -0.8:0.8
  // Scale angular velocity commands between -0.8 : 0.8
  // Give 10s to complete the motion
  if ( !jogger.jacobian_move(target_pose, 0.01, 0.05, 0.8, 0.8, ros::Duration(10)) )
  {
    ROS_ERROR_STREAM("Jacobian move failed");
    return false;
  }

  return true;
}