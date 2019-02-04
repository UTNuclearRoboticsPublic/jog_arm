//
// Created by alexander on 1/29/19.
//
#include <jog_arm/jog_calcs.h>
#include <std_msgs/Bool.h>

namespace jog_arm {

// Constructor for the class that handles jogging calculations
  JogCalcs::JogCalcs(const JogArmParameters &parameters, JogArmShared &shared_variables,
                     std::shared_ptr <robot_model_loader::RobotModelLoader> model_loader_ptr)
    : move_group_(
      parameters.move_group_name,
      boost::shared_ptr<tf::Transformer>(),
      ros::WallDuration(MOVE_GROUP_WAIT_TIMEOUT_S, 0)) {
    parameters_ = parameters;

    // Publish collision status
    warning_pub_ = nh_.advertise<std_msgs::Bool>(parameters_.warning_topic, 1);

    // MoveIt Setup
    // Wait for model_loader to be non-null.
    while (ros::ok() && !model_loader_ptr) {
      ROS_WARN_THROTTLE_NAMED(5, NODE_NAME, "Waiting for a non-null robot_model_loader pointer");
      ros::Duration(0.1).sleep();
    }
    const robot_model::RobotModelPtr &kinematic_model = model_loader_ptr->getModel();
    kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
    kinematic_state_->setToDefaultValues();

    joint_model_group_ = kinematic_model->getJointModelGroup(parameters_.move_group_name);

    std::vector<double> dummy_joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, dummy_joint_values);

    // Wait for initial messages
    ROS_INFO_NAMED(NODE_NAME, "Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>(parameters_.joint_topic);
    ROS_INFO_NAMED(NODE_NAME, "Received first joint msg.");

    ROS_INFO_NAMED(NODE_NAME, "Waiting for first command msg.");
    ros::topic::waitForMessage<geometry_msgs::TwistStamped>(parameters_.cartesian_command_in_topic);
    ROS_INFO_NAMED(NODE_NAME, "Received first command msg.");

    resetVelocityFilters();

    jt_state_.name = move_group_.getJointNames();
    jt_state_.position.resize(jt_state_.name.size());
    jt_state_.velocity.resize(jt_state_.name.size());
    jt_state_.effort.resize(jt_state_.name.size());

    // Low-pass filters for the joint positions & velocities
    for (size_t i = 0; i < jt_state_.name.size(); ++i) {
      velocity_filters_.emplace_back(parameters_.low_pass_filter_coeff);
      position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
    }

    // Initialize the position filters to initial robot joints
    while (!updateJoints() && ros::ok()) {
      incoming_jts_ = shared_variables.joints;
      ros::Duration(0.001).sleep();
    }
    for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
      position_filters_[i].reset(jt_state_.position[i]);

    // Wait for the first jogging cmd.
    // Store it in a class member for further calcs.
    // Then free up the shared variable again.
    geometry_msgs::TwistStamped cartesian_deltas;
    jog_msgs::JogJoint joint_deltas;

    while (ros::ok() && cartesian_deltas.header.stamp == ros::Time(0.) && joint_deltas.header.stamp == ros::Time(0.)) {
      ros::Duration(0.05).sleep();

      cartesian_deltas = shared_variables.command_deltas;
      joint_deltas = shared_variables.joint_command_deltas;
    }

    // Track the number of cycles during which motion has not occurred.
    // Will avoid re-publishing zero velocities endlessly.
    int zero_velocity_count = 0;

    // sleep between calcs
    const double jog_calc_sleep_time = parameters_.publish_period / 2;
    const int num_real_zero_cycles = NUM_ZERO_CYCLES_TO_PUBLISH * 2;

    // Now do jogging calcs
    while (ros::ok()) {
      // Add a small sleep to avoid 100% CPU usage
      ros::Duration(jog_calc_sleep_time).sleep();

      // If user commands are all zero, reset the low-pass filters
      // when commands resume
      bool zero_cartesian_traj_flag = shared_variables.zero_cartesian_cmd_flag;
      bool zero_joint_traj_flag = shared_variables.zero_joint_cmd_flag;
      if (shared_variables.command_is_stale) {
        zero_cartesian_traj_flag = true;
        zero_joint_traj_flag = true;
      }
      const bool zero_trajectory = zero_cartesian_traj_flag && zero_joint_traj_flag;

      if (zero_trajectory)
        // Reset low-pass filters
        resetVelocityFilters();

      // Pull data from the shared variables.
      incoming_jts_ = shared_variables.joints;

      // Initialize the position filters to initial robot joints
      while (!updateJoints() && ros::ok()) {
        incoming_jts_ = shared_variables.joints;
        ros::Duration(0.001).sleep();
      }

      // If there have not been several consecutive cycles of all zeros and joint
      // jogging commands are empty
      if (!zero_cartesian_traj_flag) {
        cartesian_deltas = shared_variables.command_deltas;

        if (!cartesianJogCalcs(cartesian_deltas, shared_variables))
          continue;
      }
        // If there have not been several consecutive cycles of all zeros and joint
        // jogging commands are not empty
      else if (!zero_joint_traj_flag) {
        joint_deltas = shared_variables.joint_command_deltas;

        if (!jointJogCalcs(joint_deltas, shared_variables))
          continue;
      }

      // if not traj has been calculated so far, try again
      if (new_traj_.joint_names.empty())
        continue;

      // Halt if the command is stale or inputs are all zero, or commands were
      // zero
      if (zero_trajectory) {
        halt(new_traj_);
      }

      // Send the newest target joints
      // If everything normal, share the new traj to be published
      if (!zero_trajectory || (zero_velocity_count <= num_real_zero_cycles)) {
        pthread_mutex_lock(&shared_variables.new_traj_mutex);
        pthread_mutex_lock(&shared_variables.ok_to_publish_mutex);
        shared_variables.new_traj = new_traj_;
        shared_variables.ok_to_publish = true;
        pthread_mutex_unlock(&shared_variables.new_traj_mutex);
        pthread_mutex_unlock(&shared_variables.ok_to_publish_mutex);
      }
        // Skip the jogging publication if all inputs have been zero for several
        // cycles in a row
      else {
        pthread_mutex_lock(&shared_variables.ok_to_publish_mutex);
        shared_variables.ok_to_publish = false;
        pthread_mutex_unlock(&shared_variables.ok_to_publish_mutex);
      }

      // Store last zero-velocity message flag to prevent superfluous warnings.
      // Cartesian and joint commands must both be zero.
      if (zero_trajectory) {
        if (zero_velocity_count <= num_real_zero_cycles) {
          zero_velocity_count += 1;
        }
      } else
        zero_velocity_count = 0;
    }
  }

// Perform the jogging calculations
  bool JogCalcs::cartesianJogCalcs(const geometry_msgs::TwistStamped &cmd, JogArmShared &shared_variables) {
    // Check for nan's in the incoming command
    if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
        std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z)) {
      ROS_WARN_STREAM_NAMED(NODE_NAME, "nan in incoming command. Skipping this datapoint.");
      return false;
    }

    // If incoming commands should be in the range [-1:1], check for |delta|>1
    if (parameters_.command_in_type == "unitless") {
      if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
          (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1)) {
        ROS_WARN_STREAM_NAMED(NODE_NAME, "Component of incoming command is >1. Skipping this datapoint.");
        return false;
      }
    }

    // Convert the cmd to the MoveGroup planning frame.
    try {
      listener_.waitForTransform(cmd.header.frame_id, parameters_.planning_frame, ros::Time::now(), ros::Duration(0.2));
    }
    catch (const tf::TransformException &ex) {
      ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << ": " << ex.what());
      return false;
    }
    // To transform, these vectors need to be stamped. See answers.ros.org
    // Q#199376
    // Transform the linear component of the cmd message
    geometry_msgs::Vector3Stamped lin_vector;
    lin_vector.vector = cmd.twist.linear;
    lin_vector.header.frame_id = cmd.header.frame_id;
    try {
      listener_.transformVector(parameters_.planning_frame, lin_vector, lin_vector);
    }
    catch (const tf::TransformException &ex) {
      ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << ": " << ex.what());
      return false;
    }

    geometry_msgs::Vector3Stamped rot_vector;
    rot_vector.vector = cmd.twist.angular;
    rot_vector.header.frame_id = cmd.header.frame_id;
    try {
      listener_.transformVector(parameters_.planning_frame, rot_vector, rot_vector);
    }
    catch (const tf::TransformException &ex) {
      ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << ": " << ex.what());
      return false;
    }

    // Put these components back into a TwistStamped
    geometry_msgs::TwistStamped twist_cmd;
    twist_cmd.header.stamp = cmd.header.stamp;
    twist_cmd.header.frame_id = parameters_.planning_frame;
    twist_cmd.twist.linear = lin_vector.vector;
    twist_cmd.twist.angular = rot_vector.vector;

    const Eigen::VectorXd delta_x = scaleCartesianCommand(twist_cmd);

    kinematic_state_->setVariableValues(jt_state_);
    original_jts_ = jt_state_;

    // Convert from cartesian commands to joint commands
    Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
    Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

    if (!addJointIncrements(jt_state_, delta_theta))
      return false;

    // Include a velocity estimate for velocity-controlled robots
    Eigen::VectorXd joint_vel(delta_theta / parameters_.publish_period);

    lowPassFilterVelocities(joint_vel);
    lowPassFilterPositions();

    const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_period);
    new_traj_ = composeOutgoingMessage(jt_state_, next_time);

    // If close to a collision or a singularity, decelerate
    applyVelocityScaling(shared_variables, new_traj_, delta_theta, decelerateForSingularity(jacobian, delta_x));

    if (!checkIfJointsWithinBounds(new_traj_)) {
      halt(new_traj_);
      publishWarning(true);
    } else
      publishWarning(false);

    // If using Gazebo simulator, insert redundant points
    if (parameters_.gazebo) {
      insertRedundantPointsIntoTrajectory(new_traj_, GAZEBO_REDUNTANT_MESSAGE_COUNT);
    }

    return true;
  }

  bool JogCalcs::jointJogCalcs(const jog_msgs::JogJoint &cmd, JogArmShared &shared_variables) {
    // Check for nan's or |delta|>1 in the incoming command
    for (const double delta : cmd.deltas) {
      if (std::isnan(delta) || (fabs(delta) > 1)) {
        ROS_WARN_STREAM_NAMED(NODE_NAME, "nan in incoming command. Skipping this datapoint.");
        return false;
      }
    }

    // Apply user-defined scaling
    const Eigen::VectorXd delta = scaleJointCommand(cmd);

    kinematic_state_->setVariableValues(jt_state_);
    original_jts_ = jt_state_;

    if (!addJointIncrements(jt_state_, delta))
      return false;

    // Include a velocity estimate for velocity-controlled robots
    Eigen::VectorXd joint_vel(delta / parameters_.publish_period);

    lowPassFilterVelocities(joint_vel);
    lowPassFilterPositions();

    // update joint state with new values
    kinematic_state_->setVariableValues(jt_state_);

    const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_delay);
    new_traj_ = composeOutgoingMessage(jt_state_, next_time);

    // check if new joint state is valid
    if (!checkIfJointsWithinBounds(new_traj_)) {
      halt(new_traj_);
      publishWarning(true);
    } else {
      publishWarning(false);
    }

    // done with calculations
    if (parameters_.gazebo) {
      insertRedundantPointsIntoTrajectory(new_traj_, GAZEBO_REDUNTANT_MESSAGE_COUNT);
    }

    return true;
  }

// Spam several redundant points into the trajectory. The first few may be
// skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo
// simulation.
// Start from 2 because the first point's timestamp is already
// 1*parameters_.publish_period
  void JogCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory &trajectory, int count) const {
    auto point = trajectory.points[0];
    // Start from 2 because we already have the first point. End at count+1 so
    // total # == count
    for (int i = 2; i < count + 1; ++i) {
      point.time_from_start = ros::Duration(i * parameters_.publish_period);
      trajectory.points.push_back(point);
    }
  }

  void JogCalcs::lowPassFilterPositions() {
    for (size_t i = 0; i < jt_state_.name.size(); ++i) {
      jt_state_.position[i] = position_filters_[i].filter(jt_state_.position[i]);

      // Check for nan's
      if (std::isnan(jt_state_.position[i])) {
        jt_state_.position[i] = original_jts_.position[i];
        jt_state_.velocity[i] = 0.;
      }
    }
  }

  void JogCalcs::lowPassFilterVelocities(const Eigen::VectorXd &joint_vel) {
    for (size_t i = 0; i < jt_state_.name.size(); ++i) {
      jt_state_.velocity[i] = velocity_filters_[i].filter(joint_vel[static_cast<long>(i)]);

      // Check for nan's
      if (std::isnan(jt_state_.velocity[static_cast<long>(i)])) {
        jt_state_.position[i] = original_jts_.position[i];
        jt_state_.velocity[i] = 0.;
        ROS_WARN_STREAM_NAMED(NODE_NAME, "nan in velocity filter");
      }
    }
  }

  trajectory_msgs::JointTrajectory JogCalcs::composeOutgoingMessage(sensor_msgs::JointState &joint_state,
                                                                    const ros::Time &stamp) const {
    trajectory_msgs::JointTrajectory new_jt_traj;
    new_jt_traj.header.frame_id = parameters_.planning_frame;
    new_jt_traj.header.stamp = stamp;
    new_jt_traj.joint_names = joint_state.name;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(parameters_.publish_period);
    if (parameters_.publish_joint_positions)
      point.positions = joint_state.position;
    if (parameters_.publish_joint_velocities)
      point.velocities = joint_state.velocity;
    if (parameters_.publish_joint_accelerations) {
      // I do not know of a robot that takes acceleration commands.
      // However, some controllers check that this data is non-empty.
      // Send all zeros, for now.
      std::vector<double> acceleration(joint_state.velocity.size());
      point.accelerations = acceleration;
    }
    new_jt_traj.points.push_back(point);

    return new_jt_traj;
  }

// Apply velocity scaling for proximity of collisions and singularities.
// Scale for collisions is read from a shared variable.
// Key equation: new_velocity =
// collision_scale*singularity_scale*previous_velocity
  bool JogCalcs::applyVelocityScaling(JogArmShared &shared_variables, trajectory_msgs::JointTrajectory &new_jt_traj,
                                      const Eigen::VectorXd &delta_theta, double singularity_scale) {
    double collision_scale = shared_variables.collision_velocity_scale;

    for (size_t i = 0; i < jt_state_.velocity.size(); ++i) {
      if (parameters_.publish_joint_positions) {
        // If close to a singularity or joint limit, undo any change to the joint
        // angles
        new_jt_traj.points[0].positions[i] =
          new_jt_traj.points[0].positions[i] -
          (1. - singularity_scale * collision_scale) * delta_theta[static_cast<long>(i)];
      }
      if (parameters_.publish_joint_velocities)
        new_jt_traj.points[0].velocities[i] *= singularity_scale * collision_scale;
    }

    return true;
  }

// Possibly calculate a velocity scaling factor, due to proximity of singularity
// and direction of motion
  double JogCalcs::decelerateForSingularity(Eigen::MatrixXd jacobian, const Eigen::VectorXd commanded_velocity) {
    double velocity_scale = 1;

    // Find the direction away from nearest singularity.
    // The last column of U from the SVD of the Jacobian points away from the
    // singularity
    Eigen::JacobiSVD <Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU);
    Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(5);

    double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

    // This singular vector tends to flip direction unpredictably. See R. Bro,
    // "Resolving the Sign Ambiguity
    // in the Singular Value Decomposition"
    // Look ahead to see if the Jacobian's condition will decrease in this
    // direction.
    // Start with a scaled version of the singular vector
    Eigen::VectorXd delta_x(6);
    double scale = 100;
    delta_x[0] = vector_toward_singularity[0] / scale;
    delta_x[1] = vector_toward_singularity[1] / scale;
    delta_x[2] = vector_toward_singularity[2] / scale;
    delta_x[3] = vector_toward_singularity[3] / scale;
    delta_x[4] = vector_toward_singularity[4] / scale;
    delta_x[5] = vector_toward_singularity[5] / scale;

    // Calculate a small change in joints
    Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

    double theta[6];
    const double *prev_joints = kinematic_state_->getVariablePositions();
    for (std::size_t i = 0, size = static_cast<std::size_t>(delta_theta.size()); i < size; ++i)
      theta[i] = prev_joints[i] + delta_theta(i);

    kinematic_state_->setVariablePositions(theta);
    jacobian = kinematic_state_->getJacobian(joint_model_group_);
    svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian);
    double new_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    // If new_condition < ini_condition, the singular vector does point towards a
    // singularity.
    //  Otherwise, flip its direction.
    if (ini_condition >= new_condition) {
      vector_toward_singularity[0] *= -1;
      vector_toward_singularity[1] *= -1;
      vector_toward_singularity[2] *= -1;
      vector_toward_singularity[3] *= -1;
      vector_toward_singularity[4] *= -1;
      vector_toward_singularity[5] *= -1;
    }

    // If this dot product is positive, we're moving toward singularity ==>
    // decelerate
    double dot = vector_toward_singularity.dot(commanded_velocity);
    if (dot > 0) {
      // Ramp velocity down linearly when the Jacobian condition is between
      // lower_singularity_threshold and
      // hard_stop_singularity_threshold, and we're moving towards the singularity
      if ((ini_condition > parameters_.lower_singularity_threshold) &&
          (ini_condition < parameters_.hard_stop_singularity_threshold)) {
        velocity_scale = 1. -
                         (ini_condition - parameters_.lower_singularity_threshold) /
                         (parameters_.hard_stop_singularity_threshold - parameters_.lower_singularity_threshold);
      }

        // Very close to singularity, so halt.
      else if (ini_condition > parameters_.hard_stop_singularity_threshold) {
        velocity_scale = 0;
        ROS_WARN_NAMED(NODE_NAME, "Close to a singularity. Halting.");
      }
    }

    return velocity_scale;
  }

  bool JogCalcs::checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory &new_jt_traj) {
    bool halting = false;
    for (auto joint : joint_model_group_->getJointModels()) {
      if (!kinematic_state_->satisfiesVelocityBounds(joint)) {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, ros::this_node::getName() << " " << joint->getName() << " "
                                                                               << " close to a "
                                                                                  " velocity limit. Enforcing limit.");
        kinematic_state_->enforceVelocityBounds(joint);
        for (std::size_t c = 0; c < new_jt_traj.joint_names.size(); ++c) {
          if (new_jt_traj.joint_names[c] == joint->getName()) {
            new_jt_traj.points[0].velocities[c] = kinematic_state_->getJointVelocities(joint)[0];
            break;
          }
        }
      }

      // Halt if we're past a joint margin and joint velocity is moving even
      // farther past
      double joint_angle = 0;
      for (std::size_t c = 0; c < new_jt_traj.joint_names.size(); ++c) {
        if (original_jts_.name[c] == joint->getName()) {
          joint_angle = original_jts_.position.at(c);
          break;
        }
      }

      if (!kinematic_state_->satisfiesPositionBounds(joint, -parameters_.joint_limit_margin)) {
        const std::vector <moveit_msgs::JointLimits> &limits = joint->getVariableBoundsMsg();

        // Joint limits are not defined for some joints. Skip them.
        if (!limits.empty()) {
          if ((kinematic_state_->getJointVelocities(joint)[0] < 0 &&
               (joint_angle <
                (limits[0].min_position + parameters_.joint_limit_margin))) ||
              (kinematic_state_->getJointVelocities(joint)[0] > 0 &&
               (joint_angle >
                (limits[0].max_position - parameters_.joint_limit_margin)))) {
            ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, ros::this_node::getName() << " " << joint->getName()
                                                                                   << " close to a "
                                                                                      " position limit. Halting.");
            halting = true;
          }
        }
      }
    }

    return !halting;
  }

  void JogCalcs::publishWarning(const bool active) const {
    std_msgs::Bool status;
    status.data = static_cast<std_msgs::Bool::_data_type>(active);
    warning_pub_.publish(status);
  }

// Avoid a singularity or other issue.
// Needs to be handled differently for position vs. velocity control
  void JogCalcs::halt(trajectory_msgs::JointTrajectory &jt_traj) {
    for (std::size_t i = 0; i < jt_state_.velocity.size(); ++i) {
      // For position-controlled robots, can reset the joints to a known, good
      // state
      if (parameters_.publish_joint_positions)
        jt_traj.points[0].positions[i] = original_jts_.position[i];

      // For velocity-controlled robots, stop
      if (parameters_.publish_joint_velocities)
        jt_traj.points[0].velocities[i] = 0;
    }
  }

// Reset the data stored in filters so the trajectory won't jump when jogging is
// resumed.
  void JogCalcs::resetVelocityFilters() {
    for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
      velocity_filters_[i].reset(0);  // Zero velocity
  }

// Parse the incoming joint msg for the joints of our MoveGroup
  bool JogCalcs::updateJoints() {
    // Check if every joint was zero. Sometimes an issue.
    bool all_zeros = true;

    // Check that the msg contains enough joints
    if (incoming_jts_.name.size() < jt_state_.name.size())
      return false;

    // Store joints in a member variable
    for (std::size_t m = 0; m < incoming_jts_.name.size(); ++m) {
      for (std::size_t c = 0; c < jt_state_.name.size(); ++c) {
        if (incoming_jts_.name[m] == jt_state_.name[c]) {
          jt_state_.position[c] = incoming_jts_.position[m];
          // Make sure there was at least one nonzero value
          if (incoming_jts_.position[m] != 0.)
            all_zeros = false;
          goto NEXT_JOINT;
        }
      }
      NEXT_JOINT:;
    }

    return !all_zeros;
  }

// Scale the incoming jog command
  Eigen::VectorXd JogCalcs::scaleCartesianCommand(const geometry_msgs::TwistStamped &command) const {
    Eigen::VectorXd result(6);

    // Apply user-defined scaling if inputs are unitless [-1:1]
    if (parameters_.command_in_type == "unitless") {
      result[0] = parameters_.linear_scale * command.twist.linear.x;
      result[1] = parameters_.linear_scale * command.twist.linear.y;
      result[2] = parameters_.linear_scale * command.twist.linear.z;
      result[3] = parameters_.rotational_scale * command.twist.angular.x;
      result[4] = parameters_.rotational_scale * command.twist.angular.y;
      result[5] = parameters_.rotational_scale * command.twist.angular.z;
    }
      // Otherwise, commands are in m/s and rad/s
    else if (parameters_.command_in_type == "speed_units") {
      result[0] = command.twist.linear.x * parameters_.publish_period;
      result[1] = command.twist.linear.y * parameters_.publish_period;
      result[2] = command.twist.linear.z * parameters_.publish_period;
      result[3] = command.twist.angular.x * parameters_.publish_period;
      result[4] = command.twist.angular.y * parameters_.publish_period;
      result[5] = command.twist.angular.z * parameters_.publish_period;
    } else
      ROS_ERROR_STREAM_NAMED(NODE_NAME, "Unexpected command_in_type");

    return result;
  }

  Eigen::VectorXd JogCalcs::scaleJointCommand(const jog_msgs::JogJoint &command) const {
    Eigen::VectorXd result(jt_state_.name.size());

    for (std::size_t i = 0; i < jt_state_.name.size(); ++i) {
      result[i] = 0.0;
    }

    // Store joints in a member variable
    for (std::size_t m = 0; m < command.joint_names.size(); ++m) {
      for (std::size_t c = 0; c < jt_state_.name.size(); ++c) {
        if (command.joint_names[m] == jt_state_.name[c]) {
          // Apply user-defined scaling if inputs are unitless [-1:1]
          if (parameters_.command_in_type == "unitless")
            result[c] = command.deltas[m] * parameters_.joint_scale;
            // Otherwise, commands are in m/s and rad/s
          else if (parameters_.command_in_type == "speed_units")
            result[c] = command.deltas[m] * parameters_.publish_period;
          else
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "Unexpected command_in_type");
          goto NEXT_JOINT;
        }
      }
      NEXT_JOINT:;
    }

    return result;
  }

// Calculate a pseudo-inverse.
  Eigen::MatrixXd JogCalcs::pseudoInverse(const Eigen::MatrixXd &J) const {
    return J.transpose() * (J * J.transpose()).inverse();
  }

// Add the deltas to each joint
  bool JogCalcs::addJointIncrements(sensor_msgs::JointState &output, const Eigen::VectorXd &increments) const {
    for (std::size_t i = 0, size = static_cast<std::size_t>(increments.size()); i < size; ++i) {
      try {
        output.position[i] += increments[static_cast<long>(i)];
      }
      catch (const std::out_of_range &e) {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << " Lengths of output and "
                                                                       "increments do not match.");
        return false;
      }
    }

    return true;
  }

} //namespace jog_arm
