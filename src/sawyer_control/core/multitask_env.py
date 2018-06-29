import abc
from collections import OrderedDict

''' Taken from vitchyr/multiworld '''

class MultitaskEnv(metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def get_goal(self):
        """
        Returns a dictionary
        """
        pass

    """
    Implement the batch-version of these functions.
    """
    @abc.abstractmethod
    def sample_goals(self, batch_size):
        """
        :param batch_size:
        :return: Returns a dictionary mapping desired goal keys to arrays of
        size BATCH_SIZE x Z, where Z depends on the key.
        """
        pass

    @abc.abstractmethod
    def compute_rewards(self, actions, obs):
        """
        :param actions: Np array of actions
        :param obs: Batch dictionary
        :return:
        """

        pass

    def sample_goal(self):
        goals = self.sample_goals(1)[0]
        return goals

    def compute_reward(self, action, obs, goal):
        actions = action[None]
        obs = obs[None]
        goal = goal[None]
        return self.compute_rewards(actions, obs, goal)[0]

    def get_diagnostics(self, *args, **kwargs):
        """
        :param rollouts: List where each element is a dictionary describing a
        rollout. Typical dictionary might look like:
        {
            'observations': np array,
            'actions': np array,
            'next_observations': np array,
            'rewards': np array,
            'terminals': np array,
            'env_infos': list of dictionaries,
            'agent_infos': list of dictionaries,
        }
        :return: OrderedDict. Statistics to save.
        """
        return OrderedDict()

    def convert_obs_to_goals(self, obs):
        raise NotImplementedError()