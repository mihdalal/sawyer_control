#!/usr/bin/env python

import rospy
from sawyer_control.srv import action

def actions_client(action_to_execute):
    rospy.wait_for_service('actions')
    try:
        execute_action = rospy.ServiceProxy('actions', action, persistent=True)
        resp = execute_action(action_to_execute)
        return resp.actions
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    for i in range(1):
        print(actions_client([3, 3, 3, 3, 3, 3, 3]))
