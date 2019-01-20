#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import intera_interface
from std_msgs.msg import Float32
from sawyer_control.srv import angle_action
from sawyer_control.configs import ros_config
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
        self._rate = 1000


        # create our limb instance
        self._limb = intera_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._des_angles = dict()

        rospy.Subscriber("desired_joint_pos", JointState, self._set_des_pos)
        rospy.Subscriber("release_spring", Float32, self._release)

        self.max_stiffness = max_stiffness
        self.time_to_maxstiffness =  time_to_max_stiffness  ######### 0.68
        self.t_release = rospy.get_time()

        self._imp_ctrl_is_active = True
        self.max_torques = {}
        for joint, torque in zip(self._limb.joint_names(), ros_config.JOINT_SPACE_IMPD_MAX_TORQUES):
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
                    self._springs[joint] = t_delta/self.time_to_maxstiffness * self.max_stiffness #note if _release is never called, then this branch will never get run
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

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()

        # calculate current forces
        for joint in self._des_angles.keys(): #per joint torque = max_stiffness*(desired_ja-current_ja) - 2*cur_vel - clipped between min and max torques
            # spring portion
            cmd[joint] = self._springs[joint] * (self._des_angles[joint] -
                                                 cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
            cmd[joint] = np.clip(cmd[joint], -self.max_torques[joint], self.max_torques[joint])
        # command new joint torques
        if self._imp_ctrl_is_active:
            self._limb.set_joint_torques(cmd)

    def attach_springs(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._des_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            self._update_forces()
            control_rate.sleep()

def main():
    # Starting node connection to ROS
    rospy.init_node('joint_space_impd')

    js = JointSprings(limb = 'right')
    # register shutdown callback
    js.attach_springs()


if __name__ == "__main__":
    main()