from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(desired=[0.97711039, 0.56662792, 0.27901027], reward='norm', safety_box=True, action_mode='position')
total = 0
act = np.array([0.1, 0, 0])*1
num_steps = 5
env.reset()
original = env._end_effector_pose()[:3]
error = 0
for i in range(num_steps):
    im = env._get_image()
    print(im)



