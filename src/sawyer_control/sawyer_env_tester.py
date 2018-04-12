from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=np.ones(7), safety_box=False, action_mode='torque')
# print(env._joint_angles())
env.reset()