#!/usr/bin/python3
import numpy as np
from railrl.exploration_strategies.ou_strategy import OUStrategy
from sawyer_control.envs.sawyer_door import SawyerDoorXYZEnv
import cv2
env = SawyerDoorXYZEnv(action_mode='joint_space_impd', position_action_scale=0.05, max_speed=0.03)
def get_img(i):
   img = env.request_image()
   img = np.array(img)
   img = img.reshape(84, 84, 3)
   cv2.imwrite('door_images/' + str(i) + '.png', img)
   return img

#env.reset()
for i in range(9001, 10000):
	delta = np.random.uniform(-0.04, 0.04, 3)
	env._position_act(delta)
	print(i)
	get_img(i)
	if i % 1500 == 0 and i != 0:
		env.reset()
	if i % 2000 == 0:
		input()
env.reset()
print(env.ik_errors)

