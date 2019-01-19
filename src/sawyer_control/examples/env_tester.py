#!/usr/bin/python3
import numpy as np
from railrl.exploration_strategies.ou_strategy import OUStrategy
from sawyer_control.envs.sawyer_door import SawyerDoorXYZEnv
import cv2
import time
env = SawyerDoorXYZEnv(action_mode='position', position_action_scale=0.05, max_speed=0.05)
def get_img(i):
   img = env.request_image()
   img = np.array(img)
   img = img.reshape(84, 84, 3)
   cv2.imwrite('door_images_4/' + str(i) + '.png', img)
   return img

# get_img('00')


#env.reset()
# actions = 0
# start = time.time()
# for i in range(100):
# 	delta = np.random.uniform(-0.04, 0.04, 3)
# 	env._position_act(delta)
# 	actions += 1
# end = time.time()
# env.reset()
# print('actions/s', actions/(end - start))
# env.reset()
k = 0
for i in range(k, 10000):
	delta = np.random.uniform(-0.04, 0.04, 3)
	env._position_act(delta)
	print(i)
	get_img(i)
	if i % 50 == 0 and i !=0:
		env.reset()

