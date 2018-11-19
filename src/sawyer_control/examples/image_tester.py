from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
import cv2
import numpy as np
from sawyer_control.core.image_env import ImageEnv
import imutils
import copy

env = ImageEnv(SawyerReachXYZEnv())
img = env.get_image(width=1000, height=1000)
startcol = 350
startrow = 200
endcol = startcol + 450
endrow = startrow + 600
img = copy.deepcopy(img[startrow:endrow, startcol:endcol])
img = cv2.resize(img, (0, 0), fx=84/450, fy=84/600, interpolation=cv2.INTER_AREA)
img = np.asarray(img).reshape(84, 84, 3)
cv2.imwrite("test.png", img)
