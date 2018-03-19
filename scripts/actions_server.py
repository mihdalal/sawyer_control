#!/usr/bin/env python

from sawyer_control.srv import *
import rospy
import intera_interface as ii

def execute_action(req):
    arm = ii.Limb('right')

    action = list(req.actions)

    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    print(action)
    arm.set_joint_torques(joint_to_values)
    return actionResponse(action)

def action_server():
    rospy.init_node('actions', anonymous=True)
    s = rospy.Service('actions', action, execute_action)
    rospy.spin()

if __name__ == "__main__":
    action_server()
