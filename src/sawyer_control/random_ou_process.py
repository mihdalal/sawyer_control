from sawyer_reaching import SawyerXYPushingImgMultitaskEnv
import numpy as np
from railrl.exploration_strategies.ou_strategy import OUStrategy
import cv2
env = SawyerXYPushingImgMultitaskEnv(action_mode='position', reward='norm', pause_on_reset=True)
ou = OUStrategy(env.action_space)
def pos():
    return env._end_effector_pose()[:3]
env.reset()
