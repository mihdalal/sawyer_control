#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import intera_interface as ii
from sawyer_control.pd_controllers.inverse_kinematics import *
from sawyer_control.configs import ros_config
joint_names = ros_config.JOINT_NAMES

def compute_joint_angle(req):
    ee_pos = req.ee_pos
    Q = ros_config.POSITION_CONTROL_EE_ORIENTATION
    pose = get_pose_stamped(ee_pos[0], ee_pos[1], ee_pos[2], Q)
    reset_angles = ros_config.RESET_ANGLES
    reset_angles = dict(zip(joint_names, reset_angles))
    ik_angles = get_joint_angles(pose, reset_angles, True, False)
    ik_angles = [ik_angles[joint] for joint in joint_names]
    return ikResponse(ik_angles)

def inverse_kinematics_server():
    rospy.init_node('ik_server', anonymous=True)
    global arm
    arm = ii.Limb('right')
    s = rospy.Service('ik', ik, compute_joint_angle)
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()
