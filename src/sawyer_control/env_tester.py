#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
from sawyer_image import ImageSawyerEnv
import numpy as np
import cv2
env = SawyerXYZReachingEnv(desired = [0, 0, 0], reward='norm', safety_box=True, action_mode='position')
env_img = ImageSawyerEnv(env)
#env_img.reset()

for i in range(500, 10000):
    dx = np.random.uniform(-0.1, 0.1)
    dy = np.random.uniform(-0.1, 0.1)
    dz = np.random.uniform(-0.1, 0.1)
    env._act(np.array([dx, dy, dz]))
    img = env_img._get_image()
    # cv2.imwrite('a.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
    cv2.imwrite('images_data_reset_100/'+ str(i) + '.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
    print(i)
    if i % 100 == 0:
        env_img.reset()

# [ 0.65861297 -0.20119689  0.51598006]
#  [0.72442144 0.21273173 0.63395238]

