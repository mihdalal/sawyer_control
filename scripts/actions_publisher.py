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
        rate = rospy.Rate(1)
        rate.sleep()
        send_action([3, 3, 3, 3, 3, 3, 3])
        # while not rospy.is_shutdown():
        #     send_action([1, 1, 1, 1, 1, 1, 1])
    except rospy.ROSInterruptException:
        pass
