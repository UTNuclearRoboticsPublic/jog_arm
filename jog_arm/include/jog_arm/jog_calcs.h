//
// Created by alexander on 1/29/19.
//

#ifndef JOG_ARM_JOG_CALCS_H
#define JOG_ARM_JOG_CALCS_H

#include <ros/node_handle.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <jog_msgs/JogJoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "jog_arm_server.h"
#include "low_pass_filter.h"


namespace jog_arm {
/**
 * Class JogCalcs - Perform the Jacobian calculations.
 */
  class JogCalcs {
  public:
    JogCalcs(const JogArmParameters &parameters, JogArmShared &shared_variables,
             std::shared_ptr <robot_model_loader::RobotModelLoader> model_loader_ptr);

  protected:
    ros::NodeHandle nh_;

    moveit::planning_interface::MoveGroupInterface move_group_;

    sensor_msgs::JointState incoming_jts_;

    bool cartesianJogCalcs(const geometry_msgs::TwistStamped &cmd, JogArmShared &shared_variables);

    bool jointJogCalcs(const jog_msgs::JogJoint &cmd, JogArmShared &shared_variables);

    // Parse the incoming joint msg for the joints of our MoveGroup
    bool updateJoints();

    Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped &command) const;

    Eigen::VectorXd scaleJointCommand(const jog_msgs::JogJoint &command) const;

    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &J) const;

    bool addJointIncrements(sensor_msgs::JointState &output, const Eigen::VectorXd &increments) const;

    // Reset the data stored in low-pass filters so the trajectory won't jump when
    // jogging is resumed.
    void resetVelocityFilters();

    // Avoid a singularity or other issue.
    // Needs to be handled differently for position vs. velocity control
    void halt(trajectory_msgs::JointTrajectory &jt_traj);

    void publishWarning(bool active) const;

    bool checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory_ <std::allocator<void>> &new_jt_traj);

    // Possibly calculate a velocity scaling factor, due to proximity of
    // singularity and direction of motion
    double decelerateForSingularity(Eigen::MatrixXd jacobian, Eigen::VectorXd commanded_velocity);

    // Apply velocity scaling for proximity of collisions and singularities
    bool applyVelocityScaling(JogArmShared &shared_variables, trajectory_msgs::JointTrajectory &new_jt_traj,
                              const Eigen::VectorXd &delta_theta, double singularity_scale);

    trajectory_msgs::JointTrajectory composeOutgoingMessage(sensor_msgs::JointState &joint_state,
                                                            const ros::Time &stamp) const;

    void lowPassFilterVelocities(const Eigen::VectorXd &joint_vel);

    void lowPassFilterPositions();

    void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory &trajectory, int count) const;

    const robot_state::JointModelGroup *joint_model_group_;

    robot_state::RobotStatePtr kinematic_state_;

    sensor_msgs::JointState jt_state_, original_jts_;
    trajectory_msgs::JointTrajectory new_traj_;

    tf::TransformListener listener_;

    std::vector <LowPassFilter> velocity_filters_;
    std::vector <LowPassFilter> position_filters_;

    ros::Publisher warning_pub_;

    JogArmParameters parameters_;
  };

} // jog_arm

#endif //JOG_ARM_JOG_CALCS_H
