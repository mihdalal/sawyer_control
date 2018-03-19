#!/usr/bin/env python
import rospy
import intera_interface as ii

from sawyer_control.msg import actions


def execute_action(action_msg):
    arm = ii.Limb('right')

    action = action_msg.torques

    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))

    arm.set_joint_torques(joint_to_values)
    print(action)

def listener():
    rospy.init_node('actions_subscriber', anonymous=True)
    rospy.Subscriber('actions_publisher', actions, execute_action)
    rospy.spin()


if __name__ == '__main__':
    listener()