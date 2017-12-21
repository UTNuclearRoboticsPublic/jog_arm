///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.cpp
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
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

#include <jog_arm_server.h>

/////////////////////////////////////////
// MAIN handles ROS subscriptions.
// A worker thread does the calculations.
/////////////////////////////////////////

// MAIN: create the worker thread and subscribe to jogging cmds and joint angles
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_arm_server");
  ros::NodeHandle n;

  // Read ROS parameters, typically from YAML file
  jog_arm::readParams(n);

  // Crunch the numbers in this thread
  pthread_t joggingThread;
  int rc = pthread_create(&joggingThread, NULL, jog_arm::joggingPipeline, 0);

  // ROS subscriptions. Share the data with the worker thread
  ros::Subscriber cmd_sub = n.subscribe( jog_arm::cmd_in_topic, 1, jog_arm::delta_cmd_cb);
  ros::Subscriber joints_sub = n.subscribe( jog_arm::joint_topic, 1, jog_arm::joints_cb);

  // Publish freshly-calculated joints to the robot
  ros::Publisher joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>(jog_arm::cmd_out_topic, 1);

  ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::joint_topic);

  while( ros::ok() )
  {
    ros::spinOnce();
    ros::Duration(jog_arm::pub_period).sleep();

    // Send the newest target joints
    pthread_mutex_lock(&jog_arm::new_traj_mutex);
    if ( jog_arm::new_traj.joint_names.size()!= 0 )
    {
      // Check for stale cmds
      if ( ros::Time::now()-jog_arm::new_traj.header.stamp < ros::Duration(jog_arm::incoming_cmd_timeout) )
      {
        joint_trajectory_pub.publish( jog_arm::new_traj );
      }
      else
      {
        ROS_WARN_STREAM("[jog_arm_server::main] Stale joint trajectory msg.");
        ROS_WARN_STREAM("[jog_arm_server::main] Did input from the controller get interrupted? Are calculations taking too long?");
      }
    }
    pthread_mutex_unlock(&jog_arm::new_traj_mutex);
  }
  
  return 0;
}

namespace jog_arm {

// A separate thread for the heavy calculations.
void *joggingPipeline(void *threadid)
{
  jog_arm::JogArmServer ja(jog_arm::move_group_name);
}

JogArmServer::JogArmServer(std::string move_group_name) :
  arm_(move_group_name)
{
  /** MoveIt Setup **/
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();

  kinematic_state_ = std::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(kinematic_model));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name);

  const std::vector<std::string> &joint_names = joint_model_group_->getJointModelNames();
  std::vector<double> dummy_joint_values;
  kinematic_state_ -> copyJointGroupPositions(joint_model_group_, dummy_joint_values);

  // Low-pass filters for the joints
  for (int i=0; i<joint_names.size(); i++ )
    filters_.push_back( jog_arm::lpf( jog_arm::low_pass_filter_coeff ));

  // Wait for initial messages
  ROS_WARN_STREAM("[JogArmServer::JogArmServer] Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::joint_topic);
  
  jt_state_.name = arm_.getJointNames();
  jt_state_.position.resize(jt_state_.name.size());
  jt_state_.velocity.resize(jt_state_.name.size());
  jt_state_.effort.resize(jt_state_.name.size());
  
  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs.
  // Then free up the shared variable again.
  while ( cmd_deltas.header.stamp == ros::Time(0.)  )
  {
    ros::Duration(0.05).sleep();
    pthread_mutex_lock(&cmd_deltas_mutex);
    cmd_deltas_ = jog_arm::cmd_deltas;
    pthread_mutex_unlock(&cmd_deltas_mutex);
  }

  // Now do jogging calcs
  while ( ros::ok() )
  {
    // Pull data from the shared variables.
    pthread_mutex_lock(&cmd_deltas_mutex);
    cmd_deltas_ = jog_arm::cmd_deltas;
    pthread_mutex_unlock(&cmd_deltas_mutex);

    pthread_mutex_lock(&joints_mutex);
    incoming_jts_ = jog_arm::joints;
    pthread_mutex_unlock(&joints_mutex);


    prev_time_ = ros::Time::now();

    updateJoints();
    jogCalcs(cmd_deltas_);

    ros::Duration(0.001).sleep();
  }
}

void JogArmServer::jogCalcs(const geometry_msgs::TwistStamped& cmd)
{
  // Convert the cmd to the MoveGroup planning frame.

  try {
    listener_.waitForTransform( cmd.header.frame_id, jog_arm::planning_frame, ros::Time::now(), ros::Duration(0.2) );
  } catch (tf::TransformException ex) {
    ROS_ERROR("JogArmServer::jogCalcs - Failed to transform command to planning frame.");
    return;
  }
  // To transform, these vectors need to be stamped. See answers.ros.org Q#199376 (Annoying! Maybe do a PR.)
  // Transform the linear component of the cmd message
  geometry_msgs::Vector3Stamped lin_vector;
  lin_vector.vector = cmd.twist.linear;
  lin_vector.header.frame_id = cmd.header.frame_id;
  listener_.transformVector(jog_arm::planning_frame, lin_vector, lin_vector);
  
  geometry_msgs::Vector3Stamped rot_vector;
  rot_vector.vector = cmd.twist.angular;
  rot_vector.header.frame_id = cmd.header.frame_id;
  listener_.transformVector(jog_arm::planning_frame, rot_vector, rot_vector);
  
  // Put these components back into a TwistStamped
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.header.stamp = cmd.header.stamp;
  twist_cmd.header.frame_id = jog_arm::planning_frame;
  twist_cmd.twist.linear = lin_vector.vector;
  twist_cmd.twist.angular = rot_vector.vector;
  
  // Apply scaling
  const Vector6d delta_x = scaleCommand(twist_cmd);

  kinematic_state_->setVariableValues(jt_state_);
  
  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
  const Eigen::VectorXd delta_theta = pseudoInverse(jacobian)*delta_x;

  if (!addJointIncrements(jt_state_, delta_theta)) {
    return;
  }

  // Check the Jacobian with these new joints. Halt before approaching a singularity.
  kinematic_state_->setVariableValues(jt_state_);
  jacobian = kinematic_state_->getJacobian(joint_model_group_);

  // Include a velocity estimate to avoid stuttery motion
  delta_t_ = (ros::Time::now() - prev_time_).toSec();
  prev_time_ = ros::Time::now();
  Eigen::VectorXd joint_vel(delta_theta/delta_t_);
  // Low-pass filter
  for (int i=0; i<jt_state_.name.size(); i++)
    joint_vel[i] = filters_[i].filter(joint_vel[i]);
  updateJointVels(jt_state_, joint_vel);

  // Compose the outgoing msg
  trajectory_msgs::JointTrajectory new_jt_traj;
  new_jt_traj.header.frame_id = jog_arm::planning_frame;
  new_jt_traj.header.stamp = twist_cmd.header.stamp;
  new_jt_traj.joint_names = jt_state_.name;
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = jt_state_.position;
  point.velocities = jt_state_.velocity;
  new_jt_traj.points.push_back(point);

  // Verify that the future Jacobian is well-conditioned before moving.
  // Slow down if very close to a singularity.
  if (!checkConditionNumber(jacobian)) {
    ROS_ERROR("[JogArmServer::jogCalcs] The arm is close to a singularity.");

    for (int i=0; i<jt_state_.velocity.size(); i++)
      new_jt_traj.points[0].velocities[i] *= 0.3;
  }

  // Share with main to be published
  pthread_mutex_lock(&jog_arm::new_traj_mutex);
  jog_arm::new_traj = new_jt_traj;
  pthread_mutex_unlock(&jog_arm::new_traj_mutex);
}

bool JogArmServer::updateJointVels(sensor_msgs::JointState &output, const Eigen::VectorXd &joint_vels) const
{
  for (std::size_t i = 0, size = joint_vels.size(); i < size; ++i) {
    try {
      output.velocity[i] = joint_vels(i);
    } catch (std::out_of_range e) {
      ROS_ERROR("[JogArmServer::updateJointVels] Vector lengths do not match.");
      return false;
    }
  }

  return true;
}

// Parse the incoming joint msg for the joints of our MoveGroup
void JogArmServer::updateJoints()
{
  // Check that the msg contains enough joints
  if (incoming_jts_.name.size() < jt_state_.name.size()) {
    ROS_WARN("[JogArmServer::jointStateCB] The joint msg does not contain enough joints.");
    return;
  }

  // Store joints in a member variable
  for (int m=0; m<incoming_jts_.name.size(); m++)
  {
    for (int c=0; c<jt_state_.name.size(); c++)
    {
      if ( incoming_jts_.name[m] == jt_state_.name[c])
      {
        jt_state_.position[c] = incoming_jts_.position[m];
        goto NEXT_JOINT;
      }
    }
NEXT_JOINT:
    ;
  }
}

JogArmServer::Vector6d JogArmServer::scaleCommand(const geometry_msgs::TwistStamped &command) const
{
  Vector6d result;
  
  result(0) = jog_arm::linear_scale*command.twist.linear.x;
  result(1) = jog_arm::linear_scale*command.twist.linear.y;
  result(2) = jog_arm::linear_scale*command.twist.linear.z;
  result(3) = jog_arm::rot_scale*command.twist.angular.x;
  result(4) = jog_arm::rot_scale*command.twist.angular.y;
  result(5) = jog_arm::rot_scale*command.twist.angular.z;
  
  return result;
}

Eigen::MatrixXd JogArmServer::pseudoInverse(const Eigen::MatrixXd &J) const
{
  Eigen::MatrixXd transpose = J.transpose();
  return transpose*(J*transpose).inverse();
}

bool JogArmServer::addJointIncrements(sensor_msgs::JointState &output, const Eigen::VectorXd &increments) const
{
  for (std::size_t i = 0, size = increments.size(); i < size; ++i) {
    try {
      output.position[i] += increments(i);
    } catch (std::out_of_range e) {
      ROS_ERROR("JogArmServer::addJointIncrements - Lengths of output and increments do not match.");
      return false;
    }
  }
  
  return true;
}

bool JogArmServer::checkConditionNumber(const Eigen::MatrixXd &matrix) const
{
  // Get Eigenvalues
  Eigen::MatrixXd::EigenvaluesReturnType eigs = matrix.eigenvalues();
  Eigen::VectorXd eig_vector = eigs.cwiseAbs();
  
  // CN = max(eigs)/min(eigs)
  double min = eig_vector.minCoeff();
  double max = eig_vector.maxCoeff();
  
  double condition_number = max/min;
  
  return (condition_number <= jog_arm::singularity_threshold);
}

// Listen to cartesian delta commands.
// Store them in a shared variable.
void delta_cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  pthread_mutex_lock(&cmd_deltas_mutex);
  jog_arm::cmd_deltas = *msg;
  pthread_mutex_unlock(&cmd_deltas_mutex);
}

// Listen to joint angles.
// Store them in a shared variable.
void joints_cb(const sensor_msgs::JointStateConstPtr& msg)
{
  pthread_mutex_lock(&joints_mutex);
  jog_arm::joints = *msg;
  pthread_mutex_unlock(&joints_mutex);
}

// Read ROS parameters, typically from YAML file
void readParams(ros::NodeHandle& n)
{
  ROS_INFO_STREAM("---------------------------------------");
  ROS_INFO_STREAM("[jog_arm_server:readParams] Parameters:");
  ROS_INFO_STREAM("---------------------------------------");
  jog_arm::move_group_name = jog_arm::getStringParam("jog_arm_server/move_group_name", n);
  ROS_INFO_STREAM("move_group_name: " << jog_arm::move_group_name);
  jog_arm::linear_scale = jog_arm::getDoubleParam("jog_arm_server/scale/linear", n);
  ROS_INFO_STREAM("linear_scale: " << jog_arm::linear_scale);
  jog_arm::rot_scale = jog_arm::getDoubleParam("jog_arm_server/scale/rotational", n);
  ROS_INFO_STREAM("rot_scale: " << jog_arm::rot_scale);
  jog_arm::low_pass_filter_coeff = jog_arm::getDoubleParam("jog_arm_server/low_pass_filter_coeff", n);
  ROS_INFO_STREAM("low_pass_filter_coeff: " << jog_arm::low_pass_filter_coeff);
  jog_arm::joint_topic = jog_arm::getStringParam("jog_arm_server/joint_topic", n);
  ROS_INFO_STREAM("joint_topic: " << jog_arm::joint_topic);
  jog_arm::cmd_in_topic = jog_arm::getStringParam("jog_arm_server/cmd_in_topic", n);
  ROS_INFO_STREAM("cmd_in_topic: " << jog_arm::cmd_in_topic);
  jog_arm::incoming_cmd_timeout = jog_arm::getDoubleParam("jog_arm_server/incoming_cmd_timeout", n);
  ROS_INFO_STREAM("incoming_cmd_timeout: " << jog_arm::incoming_cmd_timeout);
  jog_arm::cmd_out_topic = jog_arm::getStringParam("jog_arm_server/cmd_out_topic", n);
  ROS_INFO_STREAM("cmd_out_topic: " << jog_arm::cmd_out_topic);
  jog_arm::singularity_threshold = jog_arm::getDoubleParam("jog_arm_server/singularity_threshold", n);
  ROS_INFO_STREAM("singularity_threshold: " << jog_arm::singularity_threshold);
  jog_arm::planning_frame = jog_arm::getStringParam("jog_arm_server/planning_frame", n);
  ROS_INFO_STREAM("planning_frame: " << jog_arm::planning_frame);
  jog_arm::pub_period = jog_arm::getDoubleParam("jog_arm_server/pub_period", n);
  ROS_INFO_STREAM("pub_period: " << jog_arm::pub_period);
  ROS_INFO_STREAM("---------------------------------------");
  ROS_INFO_STREAM("---------------------------------------");
}

std::string getStringParam(std::string s, ros::NodeHandle& n)
{
  if( !n.getParam(s, s) )
    ROS_ERROR_STREAM("[JogArmServer::getStringParam] YAML config file does not contain parameter " << s);
  return s;
}

double getDoubleParam(std::string name, ros::NodeHandle& n)
{
  double value;
  if( !n.getParam(name, value) )
    ROS_ERROR_STREAM("[JogArmServer::getDoubleParam] YAML config file does not contain parameter " << name);
  return value;
}

} // namespace jog_arm