#!/usr/bin/python3
from sawyer_control.envs.sawyer_pushing import SawyerPushXYEnv
from sawyer_control.core.image_env import ImageEnv
import numpy as np
env = ImageEnv(SawyerPushXYEnv())
env._position_act(np.array([0.1, 0, 0]))
env.reset()
