import rospy
from std_msgs.msg import Empty
from sawyer_control.srv import ik
import numpy as np

class PositionPDController(object):
    """
    PD Controller for Moving to Neutral
    """
    def __init__(self):
        # control parameters
        self._rate = 1000  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._des_angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/right/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        self.max_stiffness = 20
        self.time_to_maxstiffness = .3
        self.t_release = rospy.get_time()

        self._imp_ctrl_is_active = True
        self._limb_joint_names = ['right_j0',
                                  'right_j1',
                                  'right_j2',
                                  'right_j3',
                                  'right_j4',
                                  'right_j5',
                                  'right_j6']

        for joint in self._limb_joint_names:
            self._springs[joint] = 30
            self._damping[joint] = 4

    def adjust_springs(self):
        for joint in list(self._des_angles.keys()):
            t_delta = rospy.get_time() - self.t_release
            if t_delta > 0:
                if t_delta < self.time_to_maxstiffness:
                    self._springs[joint] = t_delta/self.time_to_maxstiffness * self.max_stiffness
                else:
                    self._springs[joint] = self.max_stiffness
            else:
                print("warning t_delta smaller than zero!")

    def get_angles(self, ee_pos, current_joint_angles):
        return self.request_ik_angles(ee_pos, current_joint_angles)

    def _compute_pd_forces(self, current_joint_angles, current_joint_velocities, desired_ee_pos):
        """
        Computes the required to torque to be applied using the sawyer's current joint angles and joint velocities
        """

        self.adjust_springs()

        # disable cuff interaction
        if self._imp_ctrl_is_active:
            self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        des_angles = self.get_angles(desired_ee_pos, current_joint_angles)
        # calculate current forces
        self._des_angles = dict(zip(self._limb_joint_names, des_angles))
        for idx, joint in enumerate(self._limb_joint_names):
            # spring portion
            cmd[joint] = self._springs[joint] * (self._des_angles[joint] -
                                                 current_joint_angles[idx])
            # damping portion
            cmd[joint] -= self._damping[joint] * current_joint_velocities[idx]

        cmd = np.array([
            cmd[joint] for joint in self._limb_joint_names
        ])

        return cmd

    def request_ik_angles(self, ee_pos, joint_angles):
        rospy.wait_for_service('ik')
        try:
            get_joint_angles = rospy.ServiceProxy('ik', ik, persistent=True)
            resp = get_joint_angles(ee_pos, joint_angles)

            return (
                resp.joint_angles
            )
        except rospy.ServiceException as e:
            import ipdb; ipdb.set_trace()
            print(e)
