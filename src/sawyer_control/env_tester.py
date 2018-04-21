from sawyer_reaching import SawyerXYZReachingEnv

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='torque')

env.reset()

print(env._end_effector_pose()[:3])