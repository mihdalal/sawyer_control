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
                 goal_low=None,
                 goal_high=None,
                 reset_pos=None,
                 use_dynamixel=False,
                 **kwargs
                 ):
        Serializable.quick_init(self, locals())
        SawyerEnvBase.__init__(self, action_mode=action_mode, **kwargs)
        if goal_low is None:
            goal_low = self.config.POSITION_SAFETY_BOX.low
        if goal_high is None:
            goal_high = self.config.POSITION_SAFETY_BOX.high
        self.use_dynamixel = use_dynamixel
        if self.use_dynamixel:
            self.goal_space = Box(np.hstack((goal_low, np.array([min_door_angle]))),
                                  np.hstack((goal_high, np.array([max_door_angle]))), dtype=np.float32)
            self.dxl_ids = [1]
            self.dy = dxl(self.dxl_ids, config=self.config)
            self.reset_motor_pos = 0
            self.set_mode('train')
        else:
            self.goal_space = Box(goal_low, goal_high, dtype=np.float32)
        self._state_goal = None
        reset_free = self.reset_free
        self.reset_free = False
        self.reset()
        self.reset_free = reset_free

    @property
    def goal_dim(self):
        if self.use_dynamixel:
            return 4  # xyz for object position, angle for door
        else:
            return 3

    def set_to_goal(self, goal):
        ''' ONLY USE FOR DEBUGGING PURPOSES / GENERATING GOAL DATASET OF IMAGES, DOES NOT SET TO CORRECT DOOR ANGLE
            Assumes hook is in the door
        '''
        z = self._get_endeffector_pose()[:3][2]
        for _ in range(10):
            self._position_act(np.concatenate((goal[:2], [z])) - self._get_endeffector_pose()[:3])

    def _reset_robot_and_door(self):
        if not self.reset_free or self.eval_mode == 'eval':
            for i in range(15):
                self._act(np.array([-1, 0, 0]))
            for i in range(15):
                self._act(np.array([0, 0, 1]))
            # reset door
            dxl_ids = [1]
            print('RESETTING DOOR')
            self.reset_motor_pos = self.dy.reset(dxl_ids)[0]
            for i in range(15):
                self._act(self.reset_pos - self._get_endeffector_pose()[:3])

    def reset(self):
        if self.use_dynamixel:
            self._reset_robot_and_door()
        else:
            self._reset_robot()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def close_door_and_hook(self):
        eval_mode = self.eval_mode
        if self.use_dynamixel:
            self.eval_mode = 'eval'
            self._reset_robot_and_door()
            for i in range(15):
                self._position_act(np.array([1, 0, 0]))
            for i in range(15):
                self._position_act(np.array([0, 0, -1]))
            self.eval_mode = eval_mode
        else:
            reset_free = self.reset_free
            self.reset_free = True
            self._reset_robot()
            for i in range(15):
                self._position_act(np.array([1, 0, 0]))
            for i in range(15):
                self._position_act(np.array([0, 0, -1]))
            self.reset_free = reset_free

    def get_diagnostics(self, paths, prefix=''):
        return OrderedDict()

    def _get_relative_motor_pos(self):
        pos = 0
        num_iters = 25
        for i in range(num_iters):
            dxl_ids = [1]
            pos += self.dy.get_pos(dxl_ids)[0]
        pos /= num_iters
        return np.abs(pos - self.reset_motor_pos)

    def disengage_dynamixel(self):
        self.dy.engage_motor(self.dxl_ids, False)

    def get_load(self):
        return np.abs(self.dy.get_load(self.dxl_ids)[0])

    def compute_rewards(self, actions, obs, goals):
        raise NotImplementedError('Use Image based reward')

    def get_image(self, width=84, height=84):
        img = super().get_image(width=1000, height=1000)
        startcol = 350
        startrow = 200
        endcol = startcol + 300
        endrow = startrow + 600
        img = copy.deepcopy(img[startrow:endrow, startcol:endcol])
        img = cv2.resize(img, (0, 0), fx=width / 300, fy=height / 600, interpolation=cv2.INTER_AREA)
        img = np.asarray(img).reshape(width, height, 3)[::, ::, ::-1]
        return img

    def set_mode(self, mode):
        self.eval_mode = mode
