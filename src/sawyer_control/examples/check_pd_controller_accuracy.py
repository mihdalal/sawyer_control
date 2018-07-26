#!/usr/bin/python3
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import numpy as np
env = SawyerReachXYZEnv(action_mode='position')
step = np.array([.1, .1, 0])
num_steps = 5
env.reset()
for i in range(num_steps):
	prev_pose = env._get_endeffector_pose()
	env._act(step)
	error = np.linalg.norm(env._get_endeffector_pose()-prev_pose)
	print(error)