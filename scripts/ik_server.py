#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import intera_interface as ii
from inverse_kinematics import *
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion,
)
joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

def compute_joint_angle(req):
    ee_pos = req.ee_pos
    curr_joint_angles = req.joint_angles

    Q = Quaternion(
        x=ee_pos[3],
        y=ee_pos[4],
        z=ee_pos[5],
        w=ee_pos[6],
    )

    pose = get_pose_stamped(ee_pos[0], ee_pos[1], ee_pos[2], Q)
    curr_joint_angles = dict(zip(joint_names, curr_joint_angles))
    ik_angles = get_joint_angles(pose, curr_joint_angles)
    return ikResponse(ik_angles)

def inverse_kinematics_server():

    rospy.init_node('ik_server', anonymous=True)

    global arm
    arm = ii.Limb('right')

    s = rospy.Service('ik', observation, compute_joint_angle)
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()
