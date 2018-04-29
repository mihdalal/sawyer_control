from collections import deque

from src.sawyer_control.sawyer_env_base import SawyerEnv
from rllab.spaces.box import Box
import numpy as np
import rospy
from sawyer_control.srv import image

class SawyerImageEnv(SawyerEnv):
    def __init__(self, env, imsize=84, history_length=1, keep_prev=0, **kwargs):
        self.quick_init(locals())
        self.imsize = imsize
        self.image_length = self.imsize * self.imsize
        # This is torch format rather than PIL image
        self.image_shape = (self.imsize, self.imsize)
        # Flattened past image queue
        self.history_length = keep_prev + 1
        self.history = deque(maxlen=self.history_length)
        super().__init__(env, **kwargs)

    def _set_observation_space(self):
        self.observation_space = Box(low=0.0,
                                     high=1.0,
                                     shape=(self.image_size * self.history_length)
                                     )

    def _get_observation(self):
        return self._get_image()

    def _get_image(self):
        temp = self.request_image()
        img = np.array(temp)
        image = img.reshape(84, 84, 3)
        #add image flipping across horizontal
        return image

    def request_image(self):
        rospy.wait_for_service('images')
        try:
            request = rospy.ServiceProxy('images', image, persistent=True)
            obs = request()
            return (
                    obs.image
            )
        except rospy.ServiceException as e:
            print(e)
