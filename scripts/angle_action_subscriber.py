#!/usr/bin/env python
import rospy
import intera_interface as ii

from sawyer_control.msg import angle_action


def execute_action(action_msg):
    action = action_msg.angles
    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    arm.move_to_joint_positions(joint_to_values)

def listener():

    rospy.init_node('angle_action_subscriber', anonymous=True)
    rospy.Subscriber('angle_action_publisher', angle_action, execute_action)

    global arm
    arm = ii.Limb('right')

    rospy.spin()


if __name__ == '__main__':
    listener()
