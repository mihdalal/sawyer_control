#!/usr/bin/env python
import rospy
import intera_interface as ii

from sawyer_control.msg import observations


def callback(observation):
    angles = observation.angles
    velocities = observation.velocities
    torques = observation.torques
    endpoint_pose = observation.endpoint_pose

    print(angles)
    print(velocities)
    print(torques)
    print(endpoint_pose)


def listener():
    rospy.init_node('observations_subscriber')
    rospy.Subscriber('observations', observations, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()