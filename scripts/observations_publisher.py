#!/usr/bin/env python
import rospy
from sawyer_control.msg import observations
import intera_interface as ii

def observation_publisher():
    pub = rospy.Publisher('observations_publisher', observations, queue_size=10)
    rospy.init_node('observer', anonymous=True)
    rate = rospy.Rate(10)
    arm = ii.Limb('right')
    while not rospy.is_shutdown():
        angles_dict = arm.joint_angles()
        joint_names = arm.joint_names()
        angles = [
            angles_dict[joint] for joint in joint_names
        ]
        velocities_dict = arm.joint_velocities()
        velocities = [
            velocities_dict[joint] for joint in joint_names
        ]
        torques_dict = arm.joint_efforts
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
        pub.publish(angles, velocities, torques, endpoint_pose)

if __name__ == '__main__':
    try:
        observation_publisher()
    except rospy.ROSInterruptException:
        pass