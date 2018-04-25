#!/usr/bin/env python
import rospy
from sawyer_control.srv import image

def request_observation():
    rospy.init_node('ba')
    rospy.wait_for_service('images')
    try:
        get_image = rospy.ServiceProxy('images', image, persistent=True)
        obs = get_image()

        return (
                obs.image
        )
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    print(request_observation())
