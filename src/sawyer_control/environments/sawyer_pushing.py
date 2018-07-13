from collections import OrderedDict
import numpy as np
from sawyer_control.environments.sawyer_env_base import SawyerBaseEnv
from sawyer_control.core.serializable import Serializable
from sawyer_control.core.env_util import get_stat_in_paths, \
    create_stats_ordered_dict, get_asset_full_path

class SawyerXYZPushingMultitaskEnv(SawyerBaseEnv, MultitaskEnv):
    def __init__(self,
                 fixed_goal=(1, 1, 1),
                 indicator_threshold=.05,
                 reward_type='hand_distance',
                 pause_on_reset=True,
                 **kwargs
                 ):
        Serializable.quick_init(self, locals())
        MultitaskEnv.__init__(self)
        SawyerBaseEnv.__init__(self, **kwargs)
        self.goal_space = self.config.POSITION_SAFETY_BOX
        self.indicator_threshold = indicator_threshold
        self.reward_type = reward_type
        self.desired = None
        self.action_mode = 'position'

    @property
    def goal_dim(self):
        return 4 #xy for object position, xy for end effector position

    def set_to_goal(self, goal):
        print('pause')
        input()
        print('setting goal')
        obj_goal = np.concatenate((goal[:2], [self.z]))
        ee_goal = np.concatenate((goal[2:4], [self.z]))
        #PAUSE FOR INPUT:
        self.thresh=False
        self._joint_act(obj_goal-self._end_effector_pose()[:3])
        input()
        self._joint_act(ee_goal - self._end_effector_pose()[:3])
        print('setting ee')
        input()
        #PAUSE FOR INPUT
        #PLACE OBJECT AT GOAL POSITION
        #MOVES ROBOT ARM TO GOAL POSITION:
        self.desired = self._get_observation()
        self.thresh = True



    def compute_her_reward_np(self, observation, action, next_observation, goal):
        '''
        this shouldn't be used either
        :param observation:
        :param action:
        :param next_observation:
        :param goal:
        :return:
        '''
        return 0

    def reset(self):
        self.thresh = False
        self._act(self.reset_position  - self._end_effector_pose()[:3])
        self.thresh = True
        if self.pause_on_reset:
            #PAUSE AND MOVE OBJECT TO ITS RESET POSITION
            print('in reset')
            input()
        observation = self._get_observation()
        return observation

    def _get_info(self):
        return None



    def get_diagnostics(self, paths):
        return OrderedDict()
