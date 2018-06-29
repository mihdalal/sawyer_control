#!/usr/bin/python3
from sawyer_control.environments.sawyer_reaching import SawyerXYZReacher
env = SawyerXYZReacher(fix_goal=True)
env.reset()
