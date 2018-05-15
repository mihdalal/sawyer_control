#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
from sawyer_control.sawyer_image import ImageSawyerEnv

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='torque', update_hz=20)
print(env._end_effector_pose()[:3])
