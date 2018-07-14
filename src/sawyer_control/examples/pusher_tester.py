#!/usr/bin/python3
from sawyer_control.envs.sawyer_pushing import SawyerPushXYEnv
from sawyer_control.core.image_env import ImageEnv
env = ImageEnv(SawyerPushXYEnv())
env.reset()
