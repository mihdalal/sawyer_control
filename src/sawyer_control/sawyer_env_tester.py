from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=np.ones(7), safety_box=False, action_mode='position')
env.reset()
