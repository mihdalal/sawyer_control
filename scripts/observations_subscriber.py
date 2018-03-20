#!/usr/bin/env python
import rospy
import intera_interface as ii

from sawyer_control.msg import observations


def observation_callback(observation):
    angles = observation.angles
    velocities = observation.velocities
    torques = observation.torques
    endpoint_pose = observation.endpoint_pose

    print(angles)
    print(velocities)
    print(torques)
    print(endpoint_pose)


def init_observations_listener():
    rospy.init_node('observations_subscriber')
    rospy.Subscriber('observations', observations, observation_callback)
    rospy.spin()


if __name__ == '__main__':
    init_observations_listener()
