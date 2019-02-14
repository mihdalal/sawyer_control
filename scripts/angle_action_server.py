#!/usr/bin/env python
import rospy
import intera_interface as ii

from sawyer_control.scripts.impedance_controller import ImpedanceWSGController
from sawyer_control.srv import angle_action
from sawyer_control.srv import *
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# class JointController(object):
#
#     def __init__(self,
#                  limb,
#                  rate = 1000.0,
#                  ):
#
#         # control parameters
#         self._rate = rate # Hz
#
#         # create our limb instance
#         self._limb = limb
#
#         # initialize parameters
#         self.imp_ctrl_publisher = rospy.Publisher('/desired_joint_pos', JointState, queue_size=1)
#         self.imp_ctrl_release_spring_pub = rospy.Publisher('/release_spring', Float32, queue_size=10)
#
#     def publish_desired_joint_angles(self, des_joint_angles):
#         """
#         non-blocking
#         """
#         js = JointState()
#         js.name = self._limb.joint_names()
#         js.position = [des_joint_angles[n] for n in js.name]
#         self.imp_ctrl_publisher.publish(js)
#
#     def move_with_impedance(self, cmd, duration=2.0):
#         jointnames = self._limb.joint_names()
#         prev_joint = [self._limb.joint_angle(j) for j in jointnames]
#         new_joint = np.array([cmd[j] for j in jointnames])
#         control_rate = rospy.Rate(self._rate)
#         start_time = rospy.get_time()  # in seconds
#         finish_time = start_time + duration # in seconds
#
#         while rospy.get_time() < finish_time:
#             int_joints = prev_joint + (rospy.get_time()-start_time)/(finish_time-start_time)*(new_joint-prev_joint)
#             cmd = dict(list(zip(self._limb.joint_names(), list(int_joints))))
#             self.publish_desired_joint_angles(cmd)
#             control_rate.sleep()


def execute_action(action_msg):
    action = action_msg.angles
    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    duration = action_msg.duration
    ja = [joint_to_values[name] for name in arm.joint_names()]
    controller.move_with_impedance([ja], duration=duration)
    return angle_actionResponse(True)

def angle_action_server():
    rospy.init_node('angle_action_server', anonymous=True)
    global arm
    global controller
    arm = ii.Limb('right')
    arm.set_joint_position_speed(0.1)
    controller = ImpedanceWSGController(control_rate=1000, robot_name='austri', print_debug=True,)
    s = rospy.Service('angle_action', angle_action, execute_action)
    rospy.spin()

if __name__ == '__main__':
    angle_action_server()