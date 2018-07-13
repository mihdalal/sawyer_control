from collections import OrderedDict
import numpy as np
from sawyer_control.environments.sawyer_env_base import SawyerBaseEnv
from sawyer_control.core.serializable import Serializable
from sawyer_control.core.env_util import get_stat_in_paths, \
    create_stats_ordered_dict, get_asset_full_path

class SawyerXYZReachingMultitaskEnv(SawyerBaseEnv, MultitaskEnv):
    def __init__(self,
                 fixed_goal=(1, 1, 1),
                 indicator_threshold=.05,
                 reward_type='hand_distance',
                 **kwargs
                 ):
        Serializable.quick_init(self, locals())
        MultitaskEnv.__init__(self)
        SawyerBaseEnv.__init__(self, **kwargs)
        self.goal_space = self.config.POSITION_SAFETY_BOX
        self.indicator_threshold=indicator_threshold
        self.reward_type = reward_type







    @property
    def goal_dim(self):
        return 3

    def compute_rewards(self, actions, obs, goals):
        distances = np.linalg.norm(obs - goals, axis=1)
        if self.reward_type == 'hand_distance':
            r = -distances
        elif self.reward_type == 'hand_success':
            r = -(distances < self.indicator_threshold).astype(float)
        else:
            raise NotImplementedError("Invalid/no reward type.")
        return r

    def _get_info(self):
        hand_distance = np.linalg.norm(self._state_goal - self._get_endeffector_pose())
        return dict(
            hand_distance=hand_distance,
            hand_success=(hand_distance<self.indicator_threshold).astype(float)
        )

    def get_diagnostics(self, paths, prefix=''):
        statistics = OrderedDict()
        for stat_name in [
            'hand_distance',
            'hand_success',
        ]:
            stat_name = stat_name
            stat = get_stat_in_paths(paths, 'env_infos', stat_name)
            statistics.update(create_stats_ordered_dict(
                '%s%s' % (prefix, stat_name),
                stat,
                always_show_all_stats=True,
                ))
            statistics.update(create_stats_ordered_dict(
                'Final %s%s' % (prefix, stat_name),
                [s[-1] for s in stat],
                always_show_all_stats=True,
                ))