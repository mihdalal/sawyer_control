from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2

from sawyer_control.core.image_env import ImageEnv

env = ImageEnv(SawyerReachXYZEnv())
img = env.get_image()
cv2.imwrite("test.png", img)
