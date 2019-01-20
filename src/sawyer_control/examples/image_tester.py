from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2
import numpy as np
from sawyer_control.core.image_env import ImageEnv
import copy

imsize=84
env = ImageEnv(SawyerReachXYZEnv())
img = env.get_image(width=1000, height=1000)
startcol = 350
startrow = 200
endcol = startcol + 300
endrow = startrow + 600
img = copy.deepcopy(img[startrow:endrow, startcol:endcol])
img = cv2.resize(img, (0, 0), fx=imsize/300, fy=imsize/600, interpolation=cv2.INTER_AREA)
img = np.asarray(img).reshape(imsize, imsize, 3)
cv2.imwrite("test.png", img)
