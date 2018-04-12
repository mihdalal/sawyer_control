from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=np.ones(7), safety_box=False, action_mode='position')
env.reset()
original = env._end_effector_pose()
deltas = np.array([.05, -.05, -.05, 0, 0, 0, 0])
env._act(deltas)
final = env._end_effector_pose()
print(final - original)
