from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=[0.97711039, 0.56662792, 0.27901027], reward='norm', safety_box=True, action_mode='torque')
# env.reset()
#
# original = env._end_effector_pose()[:3]
# print(original)
#
# env._jac_act_damp(np.array([.1, .3, 0]))
#
# print(env._end_effector_pose()[:3]-original)
#
import ipdb; ipdb.set_trace()
# for i in range(1000):
#     print(env.reward())