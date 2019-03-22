import cv2
import numpy as np
import warnings
from PIL import Image
from gym.spaces import Box

from sawyer_control.core.serializable import Serializable


class ProxyEnv(Serializable):
    def __init__(self, wrapped_env):
        self.quick_init(locals())
        self._wrapped_env = wrapped_env

    @property
    def wrapped_env(self):
        return self._wrapped_env

    def __getattr__(self, attrname):
        if attrname == '_serializable_initialized':
            return None
        return getattr(self._wrapped_env, attrname)


class ImageEnv(ProxyEnv):
    def __init__(
            self,
            wrapped_env,
            imsize=84,
            transpose=True,
            grayscale=False,
            normalize=True,
            reward_type='wrapped_env',
            threshold=10,
    ):
        self.quick_init(locals())
        super().__init__(wrapped_env)
        self.imsize = imsize
        self.transpose = transpose
        self.grayscale = grayscale
        self.normalize = normalize

        self._render_local = False

        if grayscale:
            self.image_length = self.imsize * self.imsize
        else:
            self.image_length = 3 * self.imsize * self.imsize
        self.image_shape = (self.imsize, self.imsize)
        self._img_goal = None
        self.observation_space = Box(0, 1, (self.image_length,), dtype=np.float32)
        self.reward_type = reward_type
        self.threshold = threshold

    def step(self, action):
        obs, reward, done, info = self.wrapped_env.step(action)
        new_obs = self._get_flat_img()
        reward = self.compute_reward(action, new_obs, self._img_goal)
        return new_obs, reward, done, info

    def reset(self):
        self.wrapped_env.reset()
        env_state = self.wrapped_env.get_env_state()
        self.wrapped_env.set_to_goal(self.wrapped_env.get_goal())
        self._img_goal = self._get_flat_img()
        self.wrapped_env.set_env_state(env_state)
        return self._get_flat_img()

    def _get_flat_img(self):
        # returns the image as a torch format np array
        image_obs = self._wrapped_env.get_image()
        if self._render_local:
            cv2.imshow('env', image_obs)
            cv2.waitKey(1)
        if self.grayscale:
            image_obs = Image.fromarray(image_obs).convert('L')
            image_obs = np.array(image_obs)
        if self.normalize:
            image_obs = image_obs / 255.0
        if self.transpose:
            image_obs = image_obs.transpose()
        return image_obs.flatten()

    def enable_render(self):
        self._render_local = True

    """
    Multitask functions
    """

    def get_goal(self):
        return self._img_goal

    def sample_goals(self, batch_size):
        if batch_size > 1:
            warnings.warn("Sampling goal images is very slow")
        img_goals = np.zeros((batch_size, self.image_length))
        goals = self.wrapped_env.sample_goals(batch_size)
        for i, goal in enumerate(goals):
            self.wrapped_env.set_to_goal(goal)
            img_goals[i, :] = self._get_flat_img()
        if batch_size == 1:
            self.wrapped_env._goal = goals[0]
        return img_goals

    def compute_rewards(self, actions, obs, goals):
        achieved_goals = obs
        desired_goals = goals
        dist = np.linalg.norm(achieved_goals - desired_goals, axis=1)
        if self.reward_type == 'image_distance':
            return -dist
        elif self.reward_type == 'image_sparse':
            return (dist < self.threshold).astype(float) - 1
        elif self.reward_type == 'wrapped_env':
            return self.wrapped_env.compute_rewards(actions, obs, goals)
        else:
            raise NotImplementedError()


def normalize_image(image):
    assert image.dtype == np.uint8
    return np.float64(image) / 255.0


def unormalize_image(image):
    assert image.dtype != np.uint8
    return np.uint8(image * 255.0)
