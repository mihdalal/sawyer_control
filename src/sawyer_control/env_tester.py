from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='torque', update_hz=20)

# env.reset()
# actions = np.load('actions.npy')
# import pdb; pdb.set_trace()
#for action in actions:
    # env._act(action)
#    env._act(np.zeros(7))


# 0.57267553,  0.51366538,  0.54707706,

#array([ 0.31194451, -0.52431417, -0.02724585,

# for link in env.link_names:
#      print(env.pose_jacobian_dict[link][0])
#      print(env._compute_joint_distance_outside_box(env.pose_jacobian_dict[link][0]))
#
# #for i in range(1000):
# #    env._act(np.zeros(7))
print(env._end_effector_pose())
# print(env.desired)
# env._randomize_desired_end_effector_pose()
# print(env.desired)