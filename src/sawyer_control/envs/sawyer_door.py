from collections import OrderedDict
from gym.spaces import Box
from sawyer_control.envs.sawyer_env_base import SawyerEnvBase
from sawyer_control.core.serializable import Serializable
from sawyer_control.dynamixel.dynamixel_py import dxl
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
        if goal_low is None:
            goal_low = self.config.POSITION_SAFETY_BOX.low
        if goal_high is None:
                goal_high = self.config.POSITION_SAFETY_BOX.high
        self.goal_space = Box(np.hstack((goal_low, np.array([min_door_angle]))), np.hstack((goal_high, np.array([max_door_angle]))), dtype=np.float32)
        self._state_goal = None
        self.reset_free = reset_free
        if reset_pos is None:
            reset_pos = np.array([ 0.48526675,  0.07449275,  0.41430926])
        self.reset_pos=reset_pos
        self.dy = dxl([1])

    @property
    def goal_dim(self):
        return 4 #xyz for object position, angle for door

    def set_to_goal(self, goal):
        ''' ONLY USE FOR DEBUGGING PURPOSES / GENERATING GOAL DATASET OF IMAGES, DOES NOT SET TO CORRECT DOOR ANGLE
            Assumes hook is in the door
        '''
        z = self._get_endeffector_pose()[:3][2]
        for _ in range(10):
            self._position_act(np.concatenate((goal[:2], [z])) - self._get_endeffector_pose()[:3])

    def _reset_robot_and_door(self):
        if not self.reset_free:
            for i in range(10):
                self._position_act(np.array([-1, 0, 0]))
            for i in range(20):
                self._position_act(np.array([0, 0, 1]))
            #reset door
            dxl_ids = [1]
            self.dy.reset(dxl_ids)
            for i in range(15):
                self._position_act(self.reset_pos - self._get_endeffector_pose()[:3])

    def reset(self):
        self._reset_robot_and_door()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def close_door_and_hook(self):
        for i in range(10):
            self._position_act(np.array([-1, 0, 0]))
        for i in range(10):
            self._position_act(np.array([0, 0, 1]))
        dxl_ids = [1]
        self.dy.set_des_pos_loop(dxl_ids, 12)
        self.dy.set_des_pos_loop(dxl_ids, 1)
        for i in range(15):
            self._position_act(self.reset_pos - self._get_endeffector_pose()[:3])
        for i in range(10):
            self._position_act(np.array([1, 0, 0]))
        for i in range(10):
            self._position_act(np.array([0, 0, -1]))

    def get_diagnostics(self, paths, prefix=''):
        return OrderedDict()

    def _get_motor_pos(self):
        dxl_ids = [1]
        pos = self.dy.get_pos(dxl_ids)
        return pos[0]

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
        img = np.asarray(img).reshape(width, height, 3)[::, ::, ::-1]
        return img
