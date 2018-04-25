#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import intera_interface as ii
from sawyer_control.inverse_kinematics import *
from geometry_msgs.msg import (
    Quaternion,
)
joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

def compute_joint_angle(req):
    ee_pos = req.ee_pos
    curr_joint_angles = req.curr_joint_angles

    Q = Quaternion(
        x=ee_pos[3],
        y=ee_pos[4],
        z=ee_pos[5],
        w=ee_pos[6],
    )

    pose = get_pose_stamped(ee_pos[0], ee_pos[1], ee_pos[2], Q)
    curr_joint_angles = dict(zip(joint_names, curr_joint_angles))
    # reset_angles =  {'right_j0': 0.298009765625,
    #                         'right_j2': -0.350818359375,
    #                         'right_j4': 0.0557021484375,
    #                         'right_j3': 1.1678642578125,
    #                         'right_j1': -1.1768076171875,
    #                         'right_j6': 3.2978828125,
    #                         'right_j5': 1.3938330078125}

    reset_angles = {'right_j6': 2.9079873046875, 'right_j5': 0.4708291015625,
                            'right_j4': 0.2794638671875,
                            'right_j3': 1.80162890625,
                            'right_j2': -0.21348046875,
                            'right_j1': -0.9783408203125,
                            'right_j0': 0.0304824218}
    ik_angles = get_joint_angles(pose, reset_angles, True)
    ik_angles = [ik_angles[joint] for joint in joint_names]
    print(ik_angles)
    return ikResponse(ik_angles)

def inverse_kinematics_server():

    rospy.init_node('ik_server', anonymous=True)

    global arm
    arm = ii.Limb('right')
    s = rospy.Service('ik', ik, compute_joint_angle)
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()
