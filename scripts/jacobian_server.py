#!/usr/bin/env python

from sawyer_control.srv import *
import rospy
import numpy as np
from urdf_parser_py.urdf import URDF
import intera_interface as ii
from pykdl_utils.kdl_kinematics import KDLKinematics

joint_names = [
    '_l2',
    '_l3',
    '_l4',
    '_l5',
    '_l6',
    '_hand'
]


def handle_get_robot_pose_jacobian(req):
    poses = []
    jacobians = []
    q = arm.joint_angles()
    q = [q[req.name + '_j0'], q[req.name + '_j1'], q[req.name + '_j2'], q[req.name + '_j3'], q[req.name + '_j4'],
         q[req.name + '_j5'], q[req.name + '_j6']]
    for joint in joint_names:
        joint = req.name + joint
        pose = kin.forward(q, joint)
        pose = np.squeeze(np.asarray(pose))
        pose = [pose[0][3], pose[1][3], pose[2][3]]
        poses.extend(pose)
        jacobian = kin.jacobian(q, pose).getA()
        for i in range(3):
            jacobians.extend(jacobian[i])
    return getRobotPoseAndJacobianResponse(poses, jacobians)


def get_robot_pose_jacobian_server():
    rospy.init_node('get_robot_pose_jacobian_server')
    robot = URDF.from_parameter_server(key='robot_description')

    global kin
    global arm

    kin = KDLKinematics(robot, 'base', 'right_hand')

    arm = ii.Limb('right')

    s = rospy.Service('get_robot_pose_jacobian', getRobotPoseAndJacobian, handle_get_robot_pose_jacobian)
    rospy.spin()


if __name__ == "__main__":
    get_robot_pose_jacobian_server()
