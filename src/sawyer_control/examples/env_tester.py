#!/usr/bin/python3
from sawyer_control.environments.sawyer_reaching import SawyerReachXYZEnv
import numpy as np
env = SawyerReachXYZEnv()
env.reset()
while True:
    env.step(np.zeros(7))
