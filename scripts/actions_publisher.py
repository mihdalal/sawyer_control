#!/usr/bin/env python
import rospy
from sawyer_control.msg import actions

def actions_publisher():
    rospy.init_node('actions', anonymous=True)
    global action_publisher
    action_publisher = rospy.Publisher('actions_publisher', actions, queue_size=10)


def send_action(action):
    action_publisher.publish(action)


if __name__ == '__main__':
    try:
        actions_publisher()
        send_action([3, 3, 3, 3, 3, 3, 3])
    except rospy.ROSInterruptException:
        pass