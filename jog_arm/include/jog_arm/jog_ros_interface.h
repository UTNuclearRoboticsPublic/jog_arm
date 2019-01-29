//
// Created by alexander on 1/29/19.
//

#ifndef JOG_ARM_JOG_ROS_INTERFACE_H
#define JOG_ARM_JOG_ROS_INTERFACE_H

#include <ros/node_handle.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>
#include <jog_msgs/JogJoint.h>
#include <geometry_msgs/TwistStamped.h>
#include "jog_arm_server.h"

namespace jog_arm {

/**
 * Class JogROSInterface - Instantiated in main(). Handles ROS subs & pubs and
 * creates the worker threads.
 */
  class JogROSInterface {
  public:
    explicit JogROSInterface();

    const JogArmParameters &ros_parameters() {
      return this->ros_parameters_;
    }

    JogArmShared &shared_variables() {
      return this->shared_variables_;
    }

    std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader() {
      return this->model_loader_ptr_;
    }

  private:
    // ROS subscriber callbacks
    void deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr &msg);

    void deltaJointCmdCB(const jog_msgs::JogJointConstPtr &msg);

    void jointsCB(const sensor_msgs::JointStateConstPtr &msg);

    bool readParameters(ros::NodeHandle &n);

    // Jogging calculation thread
    static void *jogCalcThread(void *thread_id);

    // Collision checking thread
    static void *collisionCheckThread(void *thread_id);

    // Variables to share between threads
    JogArmShared shared_variables_;

    // Store the parameters that were read from ROS server
    JogArmParameters ros_parameters_;

    // static robot_model_loader::RobotModelLoader *model_loader_ptr_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr_;
  };

} // namespace jog_arm

#endif //JOG_ARM_JOG_ROS_INTERFACE_H
