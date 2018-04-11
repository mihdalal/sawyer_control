from sawyer_reaching import SawyerJointSpaceReachingEnv
import numpy as np

env = SawyerJointSpaceReachingEnv(desired=np.ones(7))
env.reset()