#!/usr/bin/python3
from sawyer_control.envs.sawyer_door import SawyerDoorEnv
env = SawyerDoorEnv()
env.reset()
print(env._get_joint_angles())
print(env._get_obs()[14:])