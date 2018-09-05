import rospy
from sawyer_control.srv import *
import numpy as np
joint_names = [
    '_l2',
    '_l3',
    '_l4',
    '_l5',
    '_l6',
    '_hand'
]

def get_pose_jacobian(poses, jacobians):
    pose_jacobian_dict = {}
    counter = 0
    pose_counter = 0
    jac_counter = 0
    for i in range(len(joint_names)):
        pose = poses[pose_counter:pose_counter+3]
        jacobian = np.array([
            jacobians[jac_counter + 3:jac_counter + 10],
            jacobians[jac_counter + 10:jac_counter + 17],
            jacobians[jac_counter + 17:jac_counter+ 24],
        ])
        pose_counter += 3
        jac_counter += 21
        pose_jacobian_dict['right' + joint_names[counter]] = [pose, jacobian]
        counter += 1
    return pose_jacobian_dict

def get_robot_pose_jacobian_client(name):
    rospy.wait_for_service('get_robot_pose_jacobian')
    try:
        get_robot_pose_jacobian = rospy.ServiceProxy('get_robot_pose_jacobian', getRobotPoseAndJacobian, persistent=True)
        resp = get_robot_pose_jacobian(name)
        d = get_pose_jacobian(resp.poses, resp.jacobians)
        return d
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    print(get_robot_pose_jacobian_client('right'))
