from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2

from sawyer_control.sawyer_image import ImageSawyerEnv

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='torque', update_hz=20)
env = ImageSawyerEnv(env)
img = env._get_image()
cv2.imwrite('img.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])