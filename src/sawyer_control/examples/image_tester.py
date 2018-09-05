from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2
<<<<<<< HEAD
import numpy as np
# env = SawyerReachXYZEnv()
# env.reset()
# img = env.get_image()
# print(img.shape)
# cv2.imshow('env', image_obs)
# cv2.waitKey(1)
=======

from sawyer_control.core.image_env import ImageEnv

env = ImageEnv(SawyerReachXYZEnv())
img = env.get_image()
cv2.imwrite("test.png", img)
>>>>>>> 2c2f4e56b732c7a9db49bc44561d69834aa22637
