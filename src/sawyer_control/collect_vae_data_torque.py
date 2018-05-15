#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
from sawyer_control.sawyer_image import ImageSawyerEnv

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='torque', update_hz=20)
env = ImageSawyerEnv(env)
for i in range(20000):
    if i % 200 == 0:
        env.reset()
    if i%2 == 0:
        action = np.random.uniform(env.joint_torque_low, env.joint_torque_high, 7)
        env._act(action)
        img = env._get_image()
        img = np.array(img)
        img = img.reshape(84, 84, 3)
        cv2.imwrite('images_data_torque_control_reset_100/'+ str(i) + '.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
