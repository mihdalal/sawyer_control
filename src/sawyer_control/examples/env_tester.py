#!/usr/bin/python3
from sawyer_control.environments.sawyer_reaching import SawyerXYZReacher
import numpy as np
env = SawyerXYZReacher(fix_goal=True)
env.reset()
while True:
    env.step(np.zeros(7))
