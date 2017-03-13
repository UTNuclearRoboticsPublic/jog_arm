///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.cpp
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

#include <jog_arm_server.h>

namespace jog_arm {

JogArmServer::JogArmServer(std::string move_group_name) :
  nh_("~"),
  arm_(move_group_name)
{
  /** Topic Setup **/
  joint_sub_ = nh_.subscribe("joint_state", 1, &JogArmServer::jointStateCB, this);

  cmd_sub_ = nh_.subscribe("cmd", 1, &JogArmServer::commandCB, this);

  /** MoveIt Setup **/
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  
  kinematic_state_ = boost::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(kinematic_model));

  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name);
}

void JogArmServer::commandCB(geometry_msgs::TwistStampedConstPtr msg)
{
  // Transform command to EEF frame, if necessary
  
  // Apply scaling
  Vector6d scaling_factor;
  scaling_factor << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
  const Vector6d delta_x = scaleCommand(msg->twist, scaling_factor);
  
  // MoveIt stuff needed to calculate the Jacobian
  kinematic_state_->setVariableValues(current_joints_);

  // Calculate the Jacobian
  const Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
  
  // Verify that the Jacobian is well-conditioned
  if (!checkConditionNumber(jacobian, 10)) {
    ROS_ERROR("JogArmServer::commandCB - Jacobian is ill-conditioned.");
    return;
  }
  
  // Convert from cartesian commands to joint commands
  const Eigen::VectorXd delta_theta = pseudoInverse(jacobian)*delta_x;
  
  // Add joint increments to current joints
  sensor_msgs::JointState new_theta;
  if (!addJointIncrements(new_theta, delta_theta)) {
    return;
  }
  
  // Set planning goal
  if (!arm_.setJointValueTarget(new_theta)) {
    ROS_ERROR("JogArmServer::commandCB - Failed to set joint target.");
    return;
  }
  
  arm_.move();
}

void JogArmServer::jointStateCB(sensor_msgs::JointStateConstPtr msg)
{
  current_joints_ = *msg;
}

JogArmServer::Vector6d JogArmServer::scaleCommand(const geometry_msgs::Twist command, const Vector6d& scalar) const
{
  Vector6d result;
  
  result(0) = scalar(0)*command.linear.x;
  result(1) = scalar(1)*command.linear.y;
  result(2) = scalar(2)*command.linear.z;
  result(3) = scalar(3)*command.angular.x;
  result(4) = scalar(4)*command.angular.y;
  result(5) = scalar(5)*command.angular.z;
  
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
      output.position.at(i) += increments(i);
    } catch (std::out_of_range e) {
      ROS_ERROR("JogArmServer::addJointIncrements - Lengths of JointState and increments do not match.");
      return false;
    }
  }
  
  return true;
}

bool JogArmServer::checkConditionNumber(const Eigen::MatrixXd &matrix, double threshold) const
{
  // Get Eigenvalues
  Eigen::MatrixXd::EigenvaluesReturnType eigs = matrix.eigenvalues();
  Eigen::VectorXd eig_vector = eigs.cwiseAbs();
  
  // CN = max(eigs)/min(eigs)
  double min = eig_vector.minCoeff();
  double max = eig_vector.maxCoeff();
  
  double condition_number = max/min;
  
  return (condition_number <= threshold);
}


} // namespace jog_arm

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_arm_Server");
  
  // Handle command line args. Note: ROS remapping arguments are removed by ros::init above.
  if( argc != 2 ) {
    ROS_FATAL("%s: Usage: rosrun jog_arm jog_arm_server [mov_group_name])", 
	       ros::this_node::getName().c_str() );
    
    std::cout << "Args received:\n";
    for ( int i = 0; i < argc; i++ ) {
      std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
    }
    return 1; // failure
  }
  
  std::string move_group_name(argv[1]);
  jog_arm::JogArmServer node(move_group_name);
  ros::spin();
  
  return 0;
}
