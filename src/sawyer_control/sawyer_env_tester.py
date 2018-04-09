from sawyer_reaching import SawyerJointSpaceReachingEnv
import numpy as np

env = SawyerJointSpaceReachingEnv(desired=np.ones(7), action_mode='position')
# env.reset()

deltas = np.array([0, 0, -0.1, 0, 0, 0, 0])
for i in range(10):
    env._act(deltas)


