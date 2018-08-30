#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import intera_interface
from intera_interface import CHECK_VERSION

from std_msgs.msg import Float32
# from sawyer_control.configs.base_config import MAX_TORQUES
MAX_TORQUES = 0.5 * np.array([8, 7, 6, 5, 4, 3, 2])
class JointSprings(object):
    """
    Virtual Joint Springs class for torque example.
    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server
    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb = "right",
				 max_stiffness = 100,
				 time_to_max_stiffness = 0.1
				 ):

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = intera_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._des_angles = dict()


        # verify robot is enabled
        # print("Getting robot state... ")
        # self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        # self._init_state = self._rs.state().enabled
        # print("Enabling robot... ")
        # self._rs.enable()
        # print("Running. Ctrl-c to quit")

        rospy.Subscriber("desired_joint_pos", JointState, self._set_des_pos)
        rospy.Subscriber("release_spring", Float32, self._release)


        self.max_stiffness = max_stiffness
        self.time_to_maxstiffness =  time_to_max_stiffness  ######### 0.68
        self.t_release = rospy.get_time()

        self._imp_ctrl_is_active = True
        self.max_torques = {}
        for joint, torque in zip(self._limb.joint_names(), MAX_TORQUES):
            self._springs[joint] = max_stiffness
            self._damping[joint] = 2
            self.max_torques[joint] = torque



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
        # if self._imp_ctrl_is_active:
        #     self._pub_cuff_disable.publish()

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
            cmd[joint] = np.clip(cmd[joint], -self.max_torques[joint], self.max_torques[joint])
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
        #self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            # if not self._rs.state().enabled:
            #     rospy.logerr("Joint torque example failed to meet "
            #                  "specified control rate timeout.")
            #     break
            self._update_forces()
            control_rate.sleep()

    # def clean_shutdown(self):
    #     """
    #     Switches out of joint torque mode to exit cleanly
    #     """
    #     print("\nExiting example...")
    #     self._limb.exit_control_mode()
    #     if not self._init_state and self._rs.state().enabled:
    #         print("Disabling robot...")
    #         self._rs.disable()


def main():


    # Starting node connection to ROS
    rospy.init_node('joint_space_impd')

    js = JointSprings(limb = 'right')
    # register shutdown callback
    #rospy.on_shutdown(js.clean_shutdown)
    js.attach_springs()


if __name__ == "__main__":
    main()