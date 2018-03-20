#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import intera_interface as ii

def get_observation(unused):
    joint_names = arm.joint_names()

    angles_dict = arm.joint_angles()
    angles = [
        angles_dict[joint] for joint in joint_names
    ]

    velocities_dict = arm.joint_velocities()
    velocities = [
        velocities_dict[joint] for joint in joint_names
    ]

    torques_dict = arm.joint_efforts()
    torques = [
        torques_dict[joint] for joint in joint_names
    ]

    state_dict = arm.endpoint_pose()
    pos = state_dict['position']
    orientation = state_dict['orientation']
    endpoint_pose = [
        pos.x,
        pos.y,
        pos.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ]

    print(angles)

    return observationResponse(angles, velocities, torques, endpoint_pose)

def observation_server():

    rospy.init_node('observation_server', anonymous=True)

    global arm
    arm = ii.Limb('right')

    s = rospy.Service('observations', observation, get_observation)
    rospy.spin()

if __name__ == "__main__":
    observation_server()
