#!/usr/bin/python3
import numpy as np
from railrl.exploration_strategies.ou_strategy import OUStrategy
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerReachXYZEnv( action_mode='joint_space_impd',
            position_action_scale=.05,
            max_speed=0.1,)
ou = OUStrategy(env.action_space, theta = 0.2)
arr = [np.array([0.05, 0, 0]),np.array([-0.05, 0, 0]) ]
env.reset()
# import time
# for i in range(250):
# 	cur = env._get_endeffector_pose()
# 	cur_time = time.time()
# 	delta = ou.get_action_from_raw_action(np.zeros(3))*0.05
# 	env._position_act(delta)
# 	new = env._get_endeffector_pose()
# 	error = delta - (new - cur)
# 	if i % 50 == 0:
# 		env.reset()
# env.reset()
# print(env.ik_errors)
