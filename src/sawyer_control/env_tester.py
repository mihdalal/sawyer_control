#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
env = SawyerXYZReachingEnv(desired = [0, 0, 0], reward='norm', safety_box=True, action_mode='position')
env.reset()
# for i in range(20000):
# 	dx = np.random.uniform(-0.1032, 0.1065)
# 	dy = np.random.uniform(-0.158, 0.13)
# 	dz = np.random.uniform(-0.32, 0)
# 	env._act(np.array([dx, dy, dz]))
# 	img = env._get_image()
# 	img = np.array(img)
# 	img = img.reshape(84, 84, 3)
# 	#v2.imwrite('a.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
# 	cv2.imwrite('images_data_1/' + 'img_i ' + str(i) + '.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
# 	env.reset()
# # img = env._get_image()
# # img = np.array(img)
# # img = img.reshape(84, 84, 3)
# # cv2.imwrite('images_data_1/' + 'img_i ' + str(0) + '.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
# # env.reset()
print(env._end_effector_pose()[:3])
