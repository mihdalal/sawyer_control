import numpy as np
from gym.spaces import Box
from sawyer_control.core.serializable import Serializable
from sawyer_control.envs.sawyer_env_base import SawyerEnvBase

class SawyerEnvBaseImage(SawyerEnvBase):
    def __init__(
            self,
            img_size=84,
            num_channels=3,
            reward_type='image_distance',
            **kwargs
    ):
        Serializable.quick_init(self, locals())
        self.img_length=num_channels*img_size**2
        self.reward_type = reward_type
        super().__init__(**kwargs)

    def step(self, action):
        self._act(action)
        observation = self._get_obs()
        reward = self.compute_reward(action, self.convert_ob_to_goal(observation), self._goal)
        info = self._get_info()
        done = False
        return observation, reward, done, info
    
    def _get_obs(self):
        image = self.get_image()
        return image

    def compute_rewards(self, actions, observations, goals):
        dist = np.linalg.norm(observations - goals, axis=1)
        if self.reward_type=='image_distance':
            return -dist
        elif self.reward_type=='image_sparse':
            return -(dist<self.threshold).astype(float)
        else:
            raise NotImplementedError()
    
    def _set_observation_space(self):
        self.observation_space = Box(0, 1, (self.img_length,))

    """
    Multitask functions
    """

    def sample_goals(self, batch_size):
        raise NotImplementedError()

    def convert_obs_to_goals(self, obs):
        return obs