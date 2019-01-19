from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2
import numpy as np
env = SawyerReachXYZEnv()
env.reset()
img = env.get_image()
print(img.shape)
cv2.imshow('env', image_obs)
cv2.waitKey(1)

