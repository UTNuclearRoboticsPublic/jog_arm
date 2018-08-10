#include <jog_msgs/JogJoint.h>
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace to_twist
{
  static const int NUM_JOINTS = 6;

  class SpaceMouseToTwist
{
public:
  SpaceMouseToTwist() :
  spinner_(2),
  joint_mode_(false),
  last_joint_mode_toggle_button_state_(false)
  {
    std::string output_topic;
    std::string joint_output_topic;
    ros::param::param("~output_topic", output_topic, std::string("jog_arm_server/delta_jog_cmds"));
    ros::param::param("~joint_output_topic", joint_output_topic, std::string("jog_arm_server/joint_delta_jog_cmds"));

    joy_sub_ = n_.subscribe("joy", 1, &SpaceMouseToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>(output_topic, 1);
    joint_delta_pub_ = n_.advertise<jog_msgs::JogJoint>(joint_output_topic, 1);

    spinner_.start();
    ros::waitForShutdown();
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::Publisher joint_delta_pub_;
  ros::AsyncSpinner spinner_;

  bool joint_mode_;
  bool last_joint_mode_toggle_button_state_;

  // Convert incoming joy commands to TwistStamped commands for jogging
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    bool joint_mode_toggle_button_state = msg->buttons[0] == 1;
    if ((joint_mode_toggle_button_state ^ last_joint_mode_toggle_button_state_) && (!joint_mode_toggle_button_state)) {
      joint_mode_ = !joint_mode_;
    }
    last_joint_mode_toggle_button_state_ = joint_mode_toggle_button_state;

    geometry_msgs::TwistStamped t_s;
    jog_msgs::JogJoint j_j;
    t_s.header.stamp = ros::Time::now();
    j_j.header.stamp = t_s.header.stamp;
    j_j.joint_names.push_back("joint_1");

    if (joint_mode_) {
      t_s.twist.linear.x = 0.0;
      t_s.twist.linear.y = 0.0;
      t_s.twist.linear.z = 0.0;

      t_s.twist.angular.x = 0.0;
      t_s.twist.angular.y = 0.0;
      t_s.twist.angular.z = 0.0;

      for (int i = 0; i < NUM_JOINTS; ++i) {
        j_j.deltas.push_back(msg->axes[i]);
      }
    }
    else {
      t_s.twist.linear.x = msg->axes[0];
      t_s.twist.linear.y = msg->axes[1];
      t_s.twist.linear.z = msg->axes[2];

      t_s.twist.angular.x = msg->axes[3];
      t_s.twist.angular.y = msg->axes[4];
      t_s.twist.angular.z = msg->axes[5];

      for (int i = 0; i < NUM_JOINTS; ++i) {
        j_j.deltas.push_back(0.0);
      }
    }

    twist_pub_.publish(t_s);
    joint_delta_pub_.publish(j_j);
  }
};
}  // end to_twist namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spacemouse_to_twist");

  to_twist::SpaceMouseToTwist to_twist;

  return 0;
}
