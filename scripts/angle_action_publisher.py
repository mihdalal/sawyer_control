#!/usr/bin/env python
import rospy
from sawyer_control.msg import angle_action

def actions_publisher():
    rospy.init_node('angle_action', anonymous=True)
    global action_publisher
    action_publisher = rospy.Publisher('angle_action_publisher', angle_action, queue_size=10)


def send_action(action):
    action_publisher.publish(action)


if __name__ == '__main__':
    try:
        actions_publisher()
        rate = rospy.Rate(1)
        rate.sleep()
        send_action([1, 1, 1, 1, 1, 1, 1])
    except rospy.ROSInterruptException:
        pass
