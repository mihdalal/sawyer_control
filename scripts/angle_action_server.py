#!/usr/bin/env python
import rospy
import intera_interface as ii
from sawyer_control.srv import angle_action
from sawyer_control.srv import *
import argparse
import importlib
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
import pdb
from std_msgs.msg import Float32
from std_msgs.msg import Int64

class JointSprings(object):
    """
    Virtual Joint Springs class for torque example.
    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server
    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self,
                    limb = arm,
                    rate = 1000.0,
                    missed_cmds = 20.0,
                    max_stiffness = 20,
                    time_to_maxstiffness = 0.3,
                 ):

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = limb

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._des_angles = dict()
        self.max_stiffness = 20
        self.time_to_maxstiffness = .3  ######### 0.68
        self.t_release = rospy.get_time()

        self._imp_ctrl_is_active = True

        for joint in self._limb.joint_names():
            self._springs[joint] = 30
            self._damping[joint] = 4

    def _imp_ctrl_active(self, inp):
        if inp.data == 1:
            print 'impedance ctrl activated'
            self._imp_ctrl_is_active = True
        if inp.data == 0:
            print 'impedance ctrl deactivated'
            self._imp_ctrl_is_active = False

    def _set_des_pos(self, jointstate):
        self._des_angles = dict(zip(jointstate.name, jointstate.position))

    def _release(self, maxstiff):
        maxstiff = maxstiff.data
        self.max_stiffness = float(maxstiff)

        print "setting maxstiffness to", maxstiff
        self.t_release = rospy.get_time()

    def adjust_springs(self):
        for joint in self._des_angles.keys():
            t_delta = rospy.get_time() - self.t_release
            if t_delta > 0:
                if t_delta < self.time_to_maxstiffness:
                    self._springs[joint] = t_delta/self.time_to_maxstiffness * self.max_stiffness
                else:
                    self._springs[joint] = self.max_stiffness
            else:
                print "warning t_delta smaller than zero!"

    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """

        # print self._springs
        self.adjust_springs()

        # disable cuff interaction
        if self._imp_ctrl_is_active:
            self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        # calculate current forces

        for joint in self._des_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self._des_angles[joint] -
                                                 cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]

        # command new joint torques
        if self._imp_ctrl_is_active:
            self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def attach_springs(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._des_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
        i = 0
        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            print(i)
            i += 1
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

def execute_action(action_msg):
    action = action_msg.angles
    thresh =action_msg.thresh
    joint_space_impd = action_msg.joint_space_impd
    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    if  joint_space_impd:
        js.attach_springs()
    else:
        if thresh:
            t = 0.5
        else:
            t = 15

        arm.move_to_joint_positions(joint_to_values, timeout = t)
        return angle_actionResponse(True)

def angle_action_server():
    rospy.init_node('angle_action_server', anonymous=True)
    global arm
    global js
    js = JointSprings()
    arm = ii.Limb('right')
    arm.set_joint_position_speed(0.15)
    s = rospy.Service('angle_action', angle_action, execute_action)
    rospy.spin()


if __name__ == '__main__':
    angle_action_server()

