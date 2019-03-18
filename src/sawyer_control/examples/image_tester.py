from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2
from sawyer_control.core.image_env import ImageEnv

env = ImageEnv(SawyerReachXYZEnv())
img = env.get_image(width=84, height=84)
cv2.imwrite("test.png", img)
