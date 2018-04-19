from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=[0.97711039, 0.56662792, 0.27901027], reward='norm', safety_box=True, action_mode='position')
total = 0
act = np.array([-.1, .1, 0])*1
num_steps = 5
# env.reset()
original = env._end_effector_pose()[:3]
print(original)
# # error = 0
# # for i in range(num_steps):
# #     original = env._end_effector_pose()[:3]
# #     env._jac_act_damp(act)
# #     total = env._end_effector_pose()[:3]-original
# #     error += (total - act)
# # print(error/num_steps)
# # print(env._end_effector_pose()[:3])
#
# total = 0
# for i in range(num_steps):
#     env.reset()
#     original = env._end_effector_pose()[:3]
#     env._jac_act_damp(act)
#     total += env._end_effector_pose()[:3]-original
# print(total/num_steps - act)