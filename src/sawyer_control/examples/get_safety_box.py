#!/usr/bin/python3
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerReachXYZEnv()
for i in range(8):
	input('move robot hand to corner of safety box')
	print(env._get_endeffector_pose())