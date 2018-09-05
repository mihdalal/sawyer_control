#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import numpy as np
import intera_interface as ii
from sawyer_control.pd_controllers.inverse_kinematics import *
from sawyer_control.configs import ros_config
joint_names = ros_config.JOINT_NAMES

def compute_joint_angle(req):
    ee_pos = req.ee_pos
    Q = Quaternion(x=-0.4939671320019262,
                   y=0.4521761317810839,
                   z=-0.4981885578001289,
                   w=0.5507643590741106
                   )



    pose = get_pose_stamped(ee_pos[0], ee_pos[1], ee_pos[2], Q)
    reset_angles = np.array([0.5679873046875, -0.788544921875, -0.6379423828125,
										1.70463671875, 0.0512109375, -0.9890234375, 0.5703583984375])
    reset_angles = dict(zip(joint_names, reset_angles))
    current_angles = arm.joint_angles()
    ik_angles = get_joint_angles(pose, reset_angles, True, True)
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
