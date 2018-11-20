from collections import OrderedDict
import numpy as np
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
        self.goal_space = Box(np.hstack((goal_low, np.array([min_door_angle]))), np.hstack((goal_high, np.array([max_door_angle]))), dtype=np.float32)
        self._state_goal = None
        self.reset_free = reset_free

    @property
    def goal_dim(self):
        return 3 #xyz for object position, angle for door

    def set_to_goal(self, goal):
        raise NotImplementedError("Hard to do because what if the hand is in "
                                  "the door? Use presampled goals.")

    def _reset_robot_and_door(self):
        if not self.reset_free:
            for i in range(15):
                self._position_act(self._get_endeffector_pose()+np.array([.1, 0, 0])) #move up
            for i in range(15):
                self._position_act(self._get_endeffector_pose()+np.array([0, 0, .1])) #move up
            for i in range(15):
                self._position_act(self._get_endeffector_pose()+np.array([-.1, 0, 0])) #move up
            self.in_reset = True
            # self._safe_move_to_neutral() #reset hand
            self.in_reset = False

    def reset(self):
        self._reset_robot_and_door()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def get_diagnostics(self, paths, prefix=''):
        return OrderedDict()

    def compute_rewards(self, actions, obs, goals):
        raise NotImplementedError('Use Image based reward')

    def get_image(self, width=84, height=84):
        img = super().get_image(width=1000, height=1000),
        startcol = 350
        startrow = 200
        endcol = startcol + 450
        endrow = startrow + 600
        img = copy.deepcopy(img[startrow:endrow, startcol:endcol])
        img = cv2.resize(img, (0, 0), fx=width/450, fy=height/600, interpolation=cv2.INTER_AREA)
        img = np.asarray(img).reshape(width, height, 3)
        return img
