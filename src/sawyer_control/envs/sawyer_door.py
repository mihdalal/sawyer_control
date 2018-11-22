from collections import OrderedDict
from gym.spaces import Box
from sawyer_control.envs.sawyer_env_base import SawyerEnvBase
from sawyer_control.core.serializable import Serializable
import numpy as np
import cv2
import copy

class SawyerDoorEnv(SawyerEnvBase):
    ''' Must Wrap with Image Env to use!'''
    def __init__(self,
                 fixed_goal=(1, 1, 1, 1),
                 action_mode='position',
                 min_door_angle=0,
                 max_door_angle=0.785398,
                 reset_free=False,
                 goal_low=None,
                 goal_high=None,
                 reset_pos=None,
                 use_state_based_door_angle=False, #must have a dynamixel to do this
                 **kwargs
                ):
        Serializable.quick_init(self, locals())
        SawyerEnvBase.__init__(self, action_mode=action_mode, **kwargs)
        if self.action_mode == 'torque':
            if goal_low is None:
                goal_low = self.config.TORQUE_SAFETY_BOX.low
            if goal_high is None:
                goal_high = self.config.TORQUE_SAFETY_BOX.high
        else:
            if goal_low is None:
                goal_low = self.config.POSITION_SAFETY_BOX.low
            if goal_high is None:
                goal_high = self.config.POSITION_SAFETY_BOX.high
        self.use_state_based_door_angle=use_state_based_door_angle
        if self.use_state_based_door_angle:
            self.goal_space = Box(np.hstack((goal_low, np.array([min_door_angle]))), np.hstack((goal_high, np.array([max_door_angle]))), dtype=np.float32)
        else:
            self.goal_space = Box(goal_low, goal_high, dtype=np.float32)
        self._state_goal = None
        self.reset_free = reset_free
        if reset_pos is None:
            reset_pos = np.array([.7,  0.03726282, 0.315])
        self.reset_pos=reset_pos

    @property
    def goal_dim(self):
        if self.use_state_based_door_angle:
            return 4 #xyz for object position, angle for door
        else:
            return 3 #xyz for object position

    def set_to_goal(self, goal):
        ''' ONLY USE FOR DEBUGGING PURPOSES / GENERATING GOAL DATASET OF IMAGES, DOES NOT SET TO CORRECT DOOR ANGLE'''
        for _ in range(10):
            self._position_act(goal[:3] - self._get_endeffector_pose()[:3])

    def _reset_robot_and_door(self):
        if not self.reset_free and self.use_state_based_door_angle:
            for i in range(15):
                self._position_act(self.reset_pos-self._get_endeffector_pose())

    def reset(self):
        self._reset_robot_and_door()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def get_diagnostics(self, paths, prefix=''):
        return OrderedDict()

    def compute_rewards(self, actions, obs, goals):
        raise NotImplementedError('Use Image based reward')

    def get_image(self, width=84, height=84):
        img = super().get_image(width=1000, height=1000)
        startcol = 350
        startrow = 200
        endcol = startcol + 450
        endrow = startrow + 600
        img = copy.deepcopy(img[startrow:endrow, startcol:endcol])
        img = cv2.resize(img, (0, 0), fx=width/450, fy=height/600, interpolation=cv2.INTER_AREA)
        img = np.asarray(img).reshape(width, height, 3)
        return img
