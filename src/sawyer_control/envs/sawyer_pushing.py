from collections import OrderedDict
import numpy as np
from gym.spaces import Box
from sawyer_control.envs.sawyer_env_base import SawyerEnvBase
from sawyer_control.core.serializable import Serializable

class SawyerPushXYEnv(SawyerEnvBase):
    ''' Must Wrap with Image Env to use!'''
    def __init__(self,
                 fixed_goal=(1, 1, 1, 1),
                 pause_on_reset=True,
                 action_mode='position',
                 z=.23128,
                 goal_low=None,
                 goal_high=None,
                 **kwargs
                ):
        Serializable.quick_init(self, locals())
        SawyerEnvBase.__init__(self, action_mode=action_mode, **kwargs)
        if goal_low is None:
            goal_low = self.config.POSITION_SAFETY_BOX.low[:2]
        if goal_high is None:
            goal_high = self.config.POSITION_SAFETY_BOX.high[:2]
        self.goal_space = Box(np.concatenate((goal_low, goal_low)), np.concatenate((goal_high, goal_high)), dtype=np.float32)
        self._state_goal = None
        self.pause_on_reset=pause_on_reset
        self.z = z

    @property
    def goal_dim(self):
        return 4 #xy for object position, xy for end effector position

    def set_to_goal(self, goal):
        print('moving arm to desired object goal')
        obj_goal = np.concatenate((goal[:2], [self.z]))
        ee_goal = np.concatenate((goal[2:4], [self.z]))
        self._position_act(obj_goal-self._get_endeffector_pose()[:3])
        input('place object at end effector location and press enter')
        self._position_act(ee_goal - self._get_endeffector_pose()[:3])

    def _reset_robot(self):
        self.in_reset = True
        self._safe_move_to_neutral()
        self.in_reset = False
        if self.pause_on_reset:
            input('move object to reset position and press enter')

    def reset(self):
        self._reset_robot()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def get_diagnostics(self, paths, prefix=''):
        return OrderedDict()

    def compute_rewards(self, actions, obs, goals):
        raise NotImplementedError('Use Image based reward')


