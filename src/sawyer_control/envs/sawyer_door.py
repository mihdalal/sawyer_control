from collections import OrderedDict
from gym.spaces import Box
from sawyer_control.envs.sawyer_env_base import SawyerEnvBase
from sawyer_control.core.serializable import Serializable
import numpy as np
import cv2


class SawyerDoorEnv(SawyerEnvBase):
    ''' Must Wrap with Image Env to use!'''

    def __init__(self,
                 fixed_goal=(1, 1, 1),
                 action_mode='position',
                 goal_low=None,
                 goal_high=None,
                 reset_free=False,
                 **kwargs
                 ):
        Serializable.quick_init(self, locals())
        SawyerEnvBase.__init__(self, action_mode=action_mode, **kwargs)
        if goal_low is None:
            goal_low = self.config.POSITION_SAFETY_BOX.low
        if goal_high is None:
            goal_high = self.config.POSITION_SAFETY_BOX.high
        self.goal_space = Box(goal_low, goal_high, dtype=np.float32)
        self._state_goal = None
        self.reset_free = reset_free

    @property
    def goal_dim(self):
        return 3

    def set_to_goal(self, goal):
        ''' ONLY USE FOR DEBUGGING PURPOSES / GENERATING GOAL DATASET OF IMAGES, DOES NOT SET TO CORRECT DOOR ANGLE
            Assumes hook is in the door
        '''
        z = self._get_endeffector_pose()[:3][2]
        for _ in range(10):
            self._position_act(np.concatenate((goal[:2], [z])) - self._get_endeffector_pose()[:3])

    def _reset_robot_and_door(self):
        for i in range(25):
            self._act(np.array([0, 0, 1])) #move arm up

        for i in range(50):
            self._act(np.array([1, 0, 0])) #push the door closed

        for i in range(50):
            self._act(np.array([-1, 0, 0]))  # pull the arm back

        for i in range(25):
            self._act(np.array([0, -1, 0]))  # move to the side

        for i in range(25):
            self._act(np.array([0, 0, -1])) #move arm down

    def reset(self):
        if not self.reset_free:
            self._reset_robot_and_door()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def close_door_and_hook(self):
        for i in range(25):
            self._act(np.array([0, 0, 1]))  # move arm up

        for i in range(50):
            self._act(np.array([1, 0, 0]))  # push the door closed

        for i in range(25):
            self._position_act(np.array([0, 0, -1])) #move arm down

    def get_diagnostics(self, paths, prefix=''):
        return OrderedDict()

    def compute_rewards(self, actions, obs, goals):
        raise NotImplementedError('Use Image based reward')

    def get_image(self, width=84, height=84):
        img = super().get_image(width=1000, height=1000)
        img = self.crop_image(img)
        img = cv2.resize(img, (0, 0), fx=width / 300, fy=height / 600, interpolation=cv2.INTER_AREA)
        img = np.asarray(img).reshape(width, height, 3)[::, ::, ::-1]
        return img
