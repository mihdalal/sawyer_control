from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2
env = SawyerReachXYZEnv()
env.reset()
img = env.get_image()
print(img.shape)
cv2.imshow('env', image_obs)
cv2.waitKey(1)
