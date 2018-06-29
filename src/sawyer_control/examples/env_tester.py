#!/usr/bin/python3
from sawyer_control.environments.sawyer_env_base import SawyerEnvBase
env = SawyerEnvBase(fix_goal=True)
# print(env._get_joint_angles())
import ipdb; ipdb.set_trace()
env.reset()
# import numpy as np
# while True:
#     env._torque_act(np.ones(7))