from sawyer_reaching import SawyerJointSpaceReachingEnv
import numpy as np

env = SawyerJointSpaceReachingEnv(desired=np.ones(7), action_mode='position')
env.reset()
print(env._end_effector_pose())
deltas = np.array([0, 0, -.1, 0, 0, 0, 0])
env._act(deltas)
print(env._end_effector_pose())

