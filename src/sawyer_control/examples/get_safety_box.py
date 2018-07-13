#!/usr/bin/python3
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import numpy as np
env = SawyerReachXYZEnv()
print(env._get_endeffector_pose())