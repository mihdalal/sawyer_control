#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
<<<<<<< HEAD
from sawyer_control.sawyer_image import ImageSawyerEnv

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='torque', update_hz=20)
env = ImageSawyerEnv(env)
img = env._get_image()
cv2.imwrite('img.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
=======
env = SawyerXYZReachingEnv(desired = [0, 0, 0], reward='norm', safety_box=True, action_mode='position')
env.reset()

for i in range(10000):
    dx = np.random.uniform(-0.03, 0.03)
    dy = np.random.uniform(-0.03, 0.03)
    dz = np.random.uniform(-0.03, 0.03)
   # print(np.array([dx, dy, dz]))
    delta = np.array([dx, dy, dz])
    cur = env._end_effector_pose()[:3]
    env._act(np.array([dx, dy, dz]))
    actual = env._end_effector_pose()[:3]
    print(actual - cur, delta)
#     img = env._get_image()
#     with open('data_vae.txt', 'a') as f:
#         f.write("%i," % (i))
#         f.write(','.join([str(x) for x in actual]))
#         f.write(','.join([str(x) for x in (actual - cur)]))
#         f.write(','.join([str(x) for x in delta]) + '\n')
#     if not img:
#         print('error')
#         break
#     if img:
#         img = np.array(img)
#         img = img.reshape(84, 84, 3)
#         #v2.imwrite('a.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
#         cv2.imwrite('images_data_1/'+ str(i) + '.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
#         #env.reset()
#         print(i)
#         if i % 100 == 0:
#             env.reset()




# # img = env._get_image()
# # img = np.array(img)
# # img = img.reshape(84, 84, 3)
# # cv2.imwrite('images_data_1/' + 'img_i ' + str(0) + '.png', img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
# # env.reset()
print(env._end_effector_pose()[:3])
>>>>>>> 9e80a8c5dcfef35984be806b00e5e81614c90585
