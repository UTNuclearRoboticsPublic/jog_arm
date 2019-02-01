//
// Created by alexander on 1/29/19.
//
#include <std_msgs/Float64MultiArray.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <jog_arm/jog_ros_interface.h>
#include <jog_arm/collision_check_thread.h>
#include <jog_arm/jog_calcs.h>

namespace jog_arm {

// Constructor for the main ROS interface node
JogROSInterface::JogROSInterface() :
  model_loader_ptr_(nullptr)
{
  ros::NodeHandle n;

  // Read ROS parameters, typically from YAML file
  if (!readParameters(n))
    exit(EXIT_FAILURE);

  // Load the robot model. This is needed by the worker threads.
  model_loader_ptr_ = std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader);

  // Crunch the numbers in this thread
  pthread_t joggingThread;
  int rc = pthread_create(&joggingThread, nullptr, jogCalcThread, this);
  if (rc)
  {
    ROS_FATAL_STREAM_NAMED(NODE_NAME, "Creating jog calculation thread failed");
    return;
  }

  // Check collisions in this thread
  pthread_t collisionThread;
  rc = pthread_create(&collisionThread, nullptr, collisionCheckThread, this);
  if (rc)
  {
    ROS_FATAL_STREAM_NAMED(NODE_NAME, "Creating collision check thread failed");
    return;
  }

  // ROS subscriptions. Share the data with the worker threads
  ros::Subscriber cmd_sub =
      n.subscribe(ros_parameters_.cartesian_command_in_topic, 1, &JogROSInterface::deltaCartesianCmdCB, this);
  ros::Subscriber joints_sub = n.subscribe(ros_parameters_.joint_topic, 1, &JogROSInterface::jointsCB, this);
  ros::Subscriber joint_jog_cmd_sub =
      n.subscribe(ros_parameters_.joint_command_in_topic, 1, &JogROSInterface::deltaJointCmdCB, this);

  // Publish freshly-calculated joints to the robot
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory
  // or std_msgs/Float64MultiArray).
  ros::Publisher outgoing_cmd_pub;
  if (ros_parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_pub = n.advertise<trajectory_msgs::JointTrajectory>(ros_parameters_.command_out_topic, 1);
  else if (ros_parameters_.command_out_type == "std_msgs/Float64MultiArray")
    outgoing_cmd_pub = n.advertise<std_msgs::Float64MultiArray>(ros_parameters_.command_out_topic, 1);

  // Wait for JointStates and clients to appear
  ROS_DEBUG_NAMED(NODE_NAME, "Waiting for JointState topic");
  ros::topic::waitForMessage<sensor_msgs::JointState>(ros_parameters_.joint_topic);
  ROS_DEBUG_NAMED(NODE_NAME, "Waiting for Cartesian command topic");
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(ros_parameters_.cartesian_command_in_topic);

  // Wait for low pass filters to stabilize
  ros::Duration(10 * ros_parameters_.publish_period).sleep();

  ros::Rate main_rate(1. / ros_parameters_.publish_period);

  while (ros::ok())
  {
    ros::spinOnce();

    pthread_mutex_lock(&shared_variables_.new_traj_mutex);
    trajectory_msgs::JointTrajectory new_traj = shared_variables_.new_traj;
    pthread_mutex_unlock(&shared_variables_.new_traj_mutex);

    // Check for stale cmds
    pthread_mutex_lock(&shared_variables_.incoming_cmd_stamp_mutex);
    {
      // Mark that incoming commands are not stale
      pthread_mutex_lock(&shared_variables_.command_is_stale_mutex);
      shared_variables_.command_is_stale = !((ros::Time::now() - shared_variables_.incoming_cmd_stamp) <
                                             ros::Duration(ros_parameters_.incoming_command_timeout));
      pthread_mutex_unlock(&shared_variables_.command_is_stale_mutex);
    }
    pthread_mutex_unlock(&shared_variables_.incoming_cmd_stamp_mutex);

    // Publish the most recent trajectory, unless the jogging calculation thread
    // tells not to
    pthread_mutex_lock(&shared_variables_.ok_to_publish_mutex);
    if (shared_variables_.ok_to_publish)
    {
      // Put the outgoing msg in the right format
      // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
      if (ros_parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
      {
        new_traj.header.stamp = ros::Time::now();
        outgoing_cmd_pub.publish(new_traj);
      }
      else if (ros_parameters_.command_out_type == "std_msgs/Float64MultiArray")
      {
        std_msgs::Float64MultiArray joints;
        if (ros_parameters_.publish_joint_positions)
          joints.data = new_traj.points[0].positions;
        else if (ros_parameters_.publish_joint_velocities)
          joints.data = new_traj.points[0].velocities;
        outgoing_cmd_pub.publish(joints);
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, "Stale or zero command. "
                                                   "Try a larger 'incoming_command_timeout' parameter?");
    }
    pthread_mutex_unlock(&shared_variables_.ok_to_publish_mutex);

    main_rate.sleep();
  }

  (void)pthread_join(joggingThread, nullptr);
  (void)pthread_join(collisionThread, nullptr);
}

// A separate thread for the heavy jogging calculations.
void* JogROSInterface::jogCalcThread(void *instance)
{
  auto interface = static_cast<JogROSInterface*>(instance);
  JogCalcs(interface->ros_parameters(), interface->shared_variables(), interface->model_loader());
  return nullptr;
}

// A separate thread for collision checking.
void* JogROSInterface::collisionCheckThread(void *instance)
{
  auto interface = static_cast<JogROSInterface*>(instance);
  CollisionCheckThread(interface->ros_parameters(), interface->shared_variables(), interface->model_loader());
  return nullptr;
}

// Listen to cartesian delta commands.
// Store them in a shared variable.
void JogROSInterface::deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.command_deltas_mutex);

  shared_variables_.command_deltas.twist = msg->twist;
  shared_variables_.command_deltas.header = msg->header;

  // Check if input is all zeros. Flag it if so to skip calculations/publication
  pthread_mutex_lock(&shared_variables_.zero_cartesian_cmd_flag_mutex);
  shared_variables_.zero_cartesian_cmd_flag = shared_variables_.command_deltas.twist.linear.x == 0.0 &&
                                              shared_variables_.command_deltas.twist.linear.y == 0.0 &&
                                              shared_variables_.command_deltas.twist.linear.z == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.x == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.y == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.z == 0.0;
  pthread_mutex_unlock(&shared_variables_.zero_cartesian_cmd_flag_mutex);

  pthread_mutex_unlock(&shared_variables_.command_deltas_mutex);

  pthread_mutex_lock(&shared_variables_.incoming_cmd_stamp_mutex);
  shared_variables_.incoming_cmd_stamp = msg->header.stamp;
  pthread_mutex_unlock(&shared_variables_.incoming_cmd_stamp_mutex);
}

// Listen to joint delta commands.
// Store them in a shared variable.
void JogROSInterface::deltaJointCmdCB(const jog_msgs::JogJointConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.joint_command_deltas_mutex);
  shared_variables_.joint_command_deltas = *msg;

  // Input frame determined by YAML file
  shared_variables_.joint_command_deltas.header.frame_id = ros_parameters_.command_frame;

  // Check if joint inputs is all zeros. Flag it if so to skip
  // calculations/publication
  bool all_zeros = true;
  for (double delta : shared_variables_.joint_command_deltas.deltas)
  {
    all_zeros &= (delta == 0.0);
  };
  pthread_mutex_unlock(&shared_variables_.joint_command_deltas_mutex);

  pthread_mutex_lock(&shared_variables_.zero_joint_cmd_flag_mutex);
  shared_variables_.zero_joint_cmd_flag = all_zeros;
  pthread_mutex_unlock(&shared_variables_.zero_joint_cmd_flag_mutex);

  pthread_mutex_lock(&shared_variables_.incoming_cmd_stamp_mutex);
  shared_variables_.incoming_cmd_stamp = msg->header.stamp;
  pthread_mutex_unlock(&shared_variables_.incoming_cmd_stamp_mutex);
}

// Listen to joint angles.
// Store them in a shared variable.
void JogROSInterface::jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.joints_mutex);
  shared_variables_.joints = *msg;
  pthread_mutex_unlock(&shared_variables_.joints_mutex);
}

// Read ROS parameters, typically from YAML file
bool JogROSInterface::readParameters(ros::NodeHandle& n)
{
  std::size_t error = 0;

  // Specified in the launch file. All other parameters will be read
  // from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);
  if (parameter_ns.empty())
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "A namespace must be specified in the launch file, like:");
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "<param name=\"parameter_ns\" "
                                      "type=\"string\" "
                                      "value=\"left_jog_arm_server\" />");
    return false;
  }

  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_period", ros_parameters_.publish_period);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_delay", ros_parameters_.publish_delay);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/collision_check_rate", ros_parameters_.collision_check_rate);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/scale/linear", ros_parameters_.linear_scale);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/scale/rotational", ros_parameters_.rotational_scale);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/scale/joint", ros_parameters_.joint_scale);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/low_pass_filter_coeff", ros_parameters_.low_pass_filter_coeff);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/joint_topic", ros_parameters_.joint_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_in_type", ros_parameters_.command_in_type);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/cartesian_command_in_topic",
                                    ros_parameters_.cartesian_command_in_topic);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/joint_command_in_topic", ros_parameters_.joint_command_in_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_frame", ros_parameters_.command_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/incoming_command_timeout",
                                    ros_parameters_.incoming_command_timeout);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/lower_singularity_threshold",
                                    ros_parameters_.lower_singularity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/hard_stop_singularity_threshold",
                                    ros_parameters_.hard_stop_singularity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/lower_collision_proximity_threshold",
                                    ros_parameters_.lower_collision_proximity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/hard_stop_collision_proximity_threshold",
                                    ros_parameters_.hard_stop_collision_proximity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/move_group_name", ros_parameters_.move_group_name);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/planning_frame", ros_parameters_.planning_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/gazebo", ros_parameters_.gazebo);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/collision_check", ros_parameters_.collision_check);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/warning_topic", ros_parameters_.warning_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/joint_limit_margin", ros_parameters_.joint_limit_margin);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_out_topic", ros_parameters_.command_out_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_out_type", ros_parameters_.command_out_type);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_positions",
                                    ros_parameters_.publish_joint_positions);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_velocities",
                                    ros_parameters_.publish_joint_velocities);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_accelerations",
                                    ros_parameters_.publish_joint_accelerations);

  rosparam_shortcuts::shutdownIfError(parameter_ns, error);

  // Set the input frame, as determined by YAML file:
  pthread_mutex_lock(&shared_variables_.command_deltas_mutex);
  shared_variables_.command_deltas.header.frame_id = ros_parameters_.command_frame;
  pthread_mutex_unlock(&shared_variables_.command_deltas_mutex);

  // Input checking
  if (ros_parameters_.hard_stop_singularity_threshold < ros_parameters_.lower_singularity_threshold)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'hard_stop_singularity_threshold' "
                              "should be greater than 'lower_singularity_threshold.' "
                              "Check yaml file.");
    return false;
  }
  if ((ros_parameters_.hard_stop_singularity_threshold < 0.) || (ros_parameters_.lower_singularity_threshold < 0.))
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameters 'hard_stop_singularity_threshold' "
                              "and 'lower_singularity_threshold' should be "
                              "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.hard_stop_collision_proximity_threshold >= ros_parameters_.lower_collision_proximity_threshold)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'hard_stop_collision_proximity_threshold' "
                              "should be less than 'lower_collision_proximity_threshold.' "
                              "Check yaml file.");
    return false;
  }
  if ((ros_parameters_.hard_stop_collision_proximity_threshold < 0.) ||
      (ros_parameters_.lower_collision_proximity_threshold < 0.))
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameters 'hard_stop_collision_proximity_threshold' "
                              "and 'lower_collision_proximity_threshold' should be "
                              "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.low_pass_filter_coeff < 0.)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'low_pass_filter_coeff' should be "
                              "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.joint_limit_margin < 0.)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'joint_limit_margin' should be "
                              "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.command_in_type != "unitless" && ros_parameters_.command_in_type != "speed_units")
  {
    ROS_WARN_NAMED(NODE_NAME, "command_in_type should be 'unitless' or "
                              "'speed_units'. Check yaml file.");
    return false;
  }
  if (ros_parameters_.command_out_type != "trajectory_msgs/JointTrajectory" &&
      ros_parameters_.command_out_type != "std_msgs/Float64MultiArray")
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter command_out_type should be "
                              "'trajectory_msgs/JointTrajectory' or "
                              "'std_msgs/Float64MultiArray'. Check yaml file.");
    return false;
  }
  if (!ros_parameters_.publish_joint_positions && !ros_parameters_.publish_joint_velocities &&
      !ros_parameters_.publish_joint_accelerations)
  {
    ROS_WARN_NAMED(NODE_NAME, "At least one of publish_joint_positions / "
                              "publish_joint_velocities / "
                              "publish_joint_accelerations must be true. Check "
                              "yaml file.");
    return false;
  }
  if ((ros_parameters_.command_out_type == "std_msgs/Float64MultiArray") && ros_parameters_.publish_joint_positions &&
      ros_parameters_.publish_joint_velocities)
  {
    ROS_WARN_NAMED(NODE_NAME, "When publishing a std_msgs/Float64MultiArray, "
                              "you must select positions OR velocities.");
    return false;
  }
  if (ros_parameters_.collision_check_rate < 0)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'collision_check_rate' should be "
                              "greater than zero. Check yaml file.");
    return false;
  }

  return true;
}

} // namespace jog_arm
