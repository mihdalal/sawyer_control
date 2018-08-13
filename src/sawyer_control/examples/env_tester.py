#!/usr/bin/python3
import numpy as np
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerReachXYZEnv(action_mode='joint_space_impd', position_action_scale=1)
arr = [np.array([0.05, 0, 0]),np.array([-0.05, 0, 0]) ]
env.reset()
import time
for i in range(100):
	cur = env._get_endeffector_pose()
	cur_time = time.time()
	delta = np.random.uniform(-0.05, 0.05, 3)
	env._position_act(delta)
	new_time = time.time()
	new = env._get_endeffector_pose()
	print("speed", np.linalg.norm(new - cur) / (new_time - cur_time))
	error = delta - (new - cur)
	print("l2", np.linalg.norm(error), "error", error)
#env.reset()

