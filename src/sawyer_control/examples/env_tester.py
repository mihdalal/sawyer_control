#!/usr/bin/python3
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerReachXYZEnv()
# env.reset()
print(env._get_joint_angles())
print(env._get_obs()[17:])