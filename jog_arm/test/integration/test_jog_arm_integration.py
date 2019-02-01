# -*- coding: utf-8 -*-
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped

from jog_msgs.msg import JogJoint
from trajectory_msgs.msg import JointTrajectory

JOINT_JOG_COMMAND_TOPIC = 'jog_arm_server/joint_delta_jog_cmds'
CARTESIAN_JOG_COMMAND_TOPIC = 'jog_arm_server/delta_jog_cmds'

COMMAND_OUT_TOPIC = 'jog_arm_server/command'


@pytest.fixture
def node():
    return rospy.init_node('pytest', anonymous=True)


class JointJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(JOINT_JOG_COMMAND_TOPIC, JogJoint, queue_size=1)

    def send_cmd(self, joint_pos):
        jj = JogJoint()
        jj.header.stamp = rospy.Time.now()
        for i in range(6):
            jj.joint_names.append('joint_{}'.format(i + 1))
            jj.deltas.append(float(joint_pos[i]))
        self._pub.publish(jj)


class CartesianJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(CARTESIAN_JOG_COMMAND_TOPIC, TwistStamped, queue_size=1)

    def send_cmd(self, linear, angular):
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x,  ts.twist.linear.y, ts.twist.linear.z = linear
        ts.twist.angular.x, ts.twist.angular.y, ts.twist.angular.z = angular
        self._pub.publish(ts)


def test_jog_arm_generates_joint_trajectory_when_joint_jog_command_is_received(node):
    # received = []
    # sub = rospy.Subscriber(COMMAND_OUT_TOPIC, JointTrajectory, lambda x: received.append(x))
    joint_cmd = JointJogCmd()
    cartesian_cmd = CartesianJogCmd()
    time.sleep(0.5)
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    joint_cmd.send_cmd([1, 0, 0, 0, 0, 0])

    msg = rospy.wait_for_message(COMMAND_OUT_TOPIC, JointTrajectory, timeout=2.0)
    # time.sleep(2.0)

    print(msg)
