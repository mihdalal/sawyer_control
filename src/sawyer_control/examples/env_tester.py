#!/usr/bin/python3
from sawyer_control.environments.sawyer_env_base import SawyerEnvBase
env = SawyerEnvBase(fix_goal=True)
env.reset()
