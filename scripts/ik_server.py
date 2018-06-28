#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import intera_interface as ii
from sawyer_control.inverse_kinematics import *
from geometry_msgs.msg import (
    Quaternion,
)
from sawyer_control.configs import ros_config
joint_names = ros_config.JOINT_NAMES

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
    curr_joint_angles = dict(zip(joint_names, curr_joint_angles)) #what is this for?
    reset_angles = base_config.RESET_JOINT_ANGLES
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
