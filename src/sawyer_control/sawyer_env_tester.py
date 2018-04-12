from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=np.ones(7), safety_box=False, action_mode='position')
env.reset()
original = env._end_effector_pose()
deltas = np.array([.2, -.3, -.5, 0, 0, 0, 0])
env._jac_act(deltas)
final = env._end_effector_pose()
print(final)
