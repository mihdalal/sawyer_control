#!/usr/bin/env python
import rospy
from sawyer_control.srv import observation

def request_observation():
    rospy.wait_for_service('observations')
    try:
        execute_action = rospy.ServiceProxy('observations', observation, persistent=True)
        obs = execute_action()

        return (
                obs.angles,
                obs.velocities,
                obs.torques,
                obs.endpoint_pose
        )
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    print(request_observation())
