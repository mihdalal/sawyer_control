#!/usr/bin/python3
import numpy as np
from railrl.exploration_strategies.ou_strategy import OUStrategy
from sawyer_control.envs.sawyer_door import SawyerDoorXYZEnv
import cv2
import time

from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
position_action_scale=.05
env = SawyerReachXYZEnv(action_mode='position', config_name='austri_config', position_action_scale=position_action_scale, max_speed=.1, use_compliant_position_controller=True)
for i in range(0, 10000):
	delta = np.random.uniform(-1, 1, 3)
	endpoint_pose_start = env._get_endeffector_pose()
	start_time = time.time()
	env._act(delta)
	end_time = time.time()
	endpoint_pose_end = env._get_endeffector_pose()
	print('Distance', np.linalg.norm(endpoint_pose_end-(endpoint_pose_start+position_action_scale*delta)))
	print('Time', end_time-start_time)
	print(i)

