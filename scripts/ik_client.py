#!/usr/bin/env python
import rospy
from sawyer_control.srv import ik
import numpy as np

def request_observation(ee_pos, joint_angles):
    rospy.wait_for_service('ik')
    try:
        get_joint_angles = rospy.ServiceProxy('ik', ik, persistent=True)
        obs = get_joint_angles(ee_pos, joint_angles)

        return (
                obs.joint_angles
        )
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    ee_pos = np.zeros(7)
    joint_angles = np.zeros(7)
    print(request_observation(ee_pos, joint_angles))
