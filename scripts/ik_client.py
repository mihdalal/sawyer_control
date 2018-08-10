#!/usr/bin/env python
import rospy
from sawyer_control.srv import ik
joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

def request_joint_angles(ee_pos, joint_angles):
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
    ee_pos = [0.4321299165321603, 0.15649043202597238, 0.9493153890114487, 0.5502556280737215, 0.7773376056237875, 0.1563899406807335, 0.26173875737106267]
    joint_angles = {'right_j6': 2.827447265625, 'right_j5': 1.8943388671875, 'right_j4': 0.425419921875, 'right_j3': 0.331033203125, 'right_j2': -0.6551962890625, 'right_j1': -1.241234375, 'right_j0': 0.140044921875}
    joint_angles = [joint_angles[joint] for joint in joint_names]
    print(request_joint_angles(ee_pos, joint_angles))
