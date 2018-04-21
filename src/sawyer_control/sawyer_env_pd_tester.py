from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

desired = [0.97711039, .5, 0.27901027]
env = SawyerXYZReachingEnv(desired=desired, reward='norm', safety_box=True, action_mode='position')
total = 0
num_steps = 5
env.reset()
error = 0
num_desireds = 10
desireds = [np.random.uniform(env.safety_box_lows, env.safety_box_highs, size=(1, 3))[0] for i in range(num_desireds)]
for desired in desireds:
    dist = 0
    for i in range(3):
        env.reset()
        for j in range(num_steps):
            original = env._end_effector_pose()[:3]
            act = desired - original
            env._act(act)

        current = env._end_effector_pose()[:3]
        dist += np.linalg.norm(desired-current)
    print(dist/3)