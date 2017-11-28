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
  jog_arm::move_group_name = jog_arm::getStringParam("jog_arm_server/move_group_name", n);
  jog_arm::linear_scale = jog_arm::getDoubleParam("jog_arm_server/scale/linear", n);
  jog_arm::rot_scale = jog_arm::getDoubleParam("jog_arm_server/scale/rotational", n);
  jog_arm::joint_topic = jog_arm::getStringParam("jog_arm_server/joint_topic", n);
  jog_arm::cmd_topic = jog_arm::getStringParam("jog_arm_server/cmd_topic", n);
  jog_arm::singularity_threshold = jog_arm::getDoubleParam("jog_arm_server/singularity_threshold", n);
  jog_arm::moveit_planning_frame = jog_arm::getStringParam("jog_arm_server/moveit_planning_frame", n);

  // Crunch the numbers in this thread
  pthread_t joggingThread;
  int rc = pthread_create(&joggingThread, NULL, jog_arm::joggingPipeline, 0);

  // ROS subscriptions. Share the data with the worker thread
  ros::Subscriber cmd_sub = n.subscribe( jog_arm::cmd_topic, 1, jog_arm::delta_cmd_cb);
  ros::Subscriber joints_sub = n.subscribe( jog_arm::joint_topic, 1, jog_arm::joints_cb);

  ros::spin();
  
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

  kinematic_state_ = boost::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(kinematic_model));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name);

  arm_.setMaxVelocityScalingFactor( 0.1 );

  const std::vector<std::string> &joint_names = joint_model_group_->getJointModelNames();
  std::vector<double> dummy_joint_values;
  kinematic_state_ -> copyJointGroupPositions(joint_model_group_, dummy_joint_values);

  // Wait for initial messages
  ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::joint_topic);
  
  current_joints_.name = arm_.getJointNames();
  current_joints_.position.resize(current_joints_.name.size());
  current_joints_.velocity.resize(current_joints_.name.size());
  current_joints_.effort.resize(current_joints_.name.size());
  
  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs.
  // Then free up the shared variable again.
  while ( cmd_deltas.header.stamp == ros::Time(0.)  )
  {
    ros::Duration(0.005).sleep();
    pthread_mutex_lock(&cmd_deltas_mutex);
    cmd_deltas_ = jog_arm::cmd_deltas;
    pthread_mutex_unlock(&cmd_deltas_mutex);
  }
  // Now do jogging calcs as quickly as possible
  while ( ros::ok() )
  {
    // Pull data from the shared variables.
    pthread_mutex_lock(&cmd_deltas_mutex);
    cmd_deltas_ = jog_arm::cmd_deltas;
    pthread_mutex_unlock(&cmd_deltas_mutex);

    pthread_mutex_lock(&joints_mutex);
    joints_ = jog_arm::joints;
    pthread_mutex_unlock(&joints_mutex);  


    updateJoints();
    jogCalcs(cmd_deltas_);

    ros::Duration(0.001).sleep();
  }
}

void JogArmServer::jogCalcs(const geometry_msgs::TwistStamped& cmd)
{
  // Convert the cmd to the MoveGroup planning frame.

  try {
    listener_.waitForTransform( cmd.header.frame_id, jog_arm::moveit_planning_frame, ros::Time::now(), ros::Duration(0.2) );
  } catch (tf::TransformException ex) {
    ROS_ERROR("JogArmServer::jogCalcs - Failed to transform command to planning frame.");
    return;
  }
  // To transform, these vectors need to be stamped. See answers.ros.org Q#199376 (Annoying! Maybe do a PR.)
  // Transform the linear component of the cmd message
  geometry_msgs::Vector3Stamped lin_vector;
  lin_vector.vector = cmd.twist.linear;
  lin_vector.header.frame_id = cmd.header.frame_id;
  listener_.transformVector(jog_arm::moveit_planning_frame, lin_vector, lin_vector);
  
  geometry_msgs::Vector3Stamped rot_vector;
  rot_vector.vector = cmd.twist.angular;
  rot_vector.header.frame_id = cmd.header.frame_id;
  listener_.transformVector(jog_arm::moveit_planning_frame, rot_vector, rot_vector);
  
  // Put these components back into a TwistStamped
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.header.frame_id = jog_arm::moveit_planning_frame;
  twist_cmd.twist.linear = lin_vector.vector;
  twist_cmd.twist.angular = rot_vector.vector;
  
  // Apply scaling
  const Vector6d delta_x = scaleCommand(twist_cmd);

  kinematic_state_->setVariableValues(current_joints_);
  
  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
  const Eigen::VectorXd delta_theta = pseudoInverse(jacobian)*delta_x;

  // Add joint increments to current joints
  sensor_msgs::JointState new_theta = current_joints_;
  if (!addJointIncrements(new_theta, delta_theta)) {
    return;
  }

  // Check the Jacobian with these new joints. Halt before approaching a singularity.
  kinematic_state_->setVariableValues(new_theta);
  jacobian = kinematic_state_->getJacobian(joint_model_group_);

  // Verify that the future Jacobian is well-conditioned before moving
  if (!checkConditionNumber(jacobian)) {
    ROS_ERROR("[JogArmServer::jogCalcs] The arm is close to a singularity.");
    return;
  }
  
  // Set planning goal
  if (!arm_.setJointValueTarget(new_theta)) {
    ROS_ERROR("JogArmServer::jogCalcs - Failed to set joint target.");
    return;
  }

  if (!arm_.move()) {
   ROS_ERROR("JogArmServer::jogCalcs - Jogging movement failed.");
   return;
  }
}

void JogArmServer::updateJoints()
{
  // Check that the msg contains enough joints
  if (joints_.name.size() < current_joints_.name.size()) {
    ROS_WARN("[JogArmServer::jointStateCB] The joint msg does not contain enough joints.");
    return;
  }

  // Store joints in a member variable
  for (int m=0; m<joints_.name.size(); m++)
  {
    for (int c=0; c<current_joints_.name.size(); c++)
    {
      if ( joints_.name[m] == current_joints_.name[c])
      {
        current_joints_.position[c] = joints_.position[m];
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