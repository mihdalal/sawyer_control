from geometry_msgs.msg import Quaternion
import numpy as np
#JOINT_CONTROLLER_SETTINGS
JOINT_POSITION_SPEED = .1
JOINT_POSITION_TIMEOUT = .5

#JOINT INFO
JOINT_NAMES = ['right_j0',
               'right_j1',
               'right_j2',
               'right_j3',
               'right_j4',
               'right_j5',
               'right_j6'
               ]
LINK_NAMES = ['right_l2', 'right_l3', 'right_l4', 'right_l5', 'right_l6', '_hand']
# RESET_ANGLES = np.array(
#     [-2.06738291e-03, - 1.17774510e+00, - 2.17089849e-03,  2.17807722e+00,
#      - 4.32617177e-04,  5.66576183e-01,  3.31357336e+00]
# )

RESET_ANGLES = np.array(
    [-0.07284961, - 0.99221385, - 0.29631543,  1.78041506, - 0.4403457, - 0.91421682,
     0.7003721]
)

RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))

# POSITION_CONTROL_EE_ORIENTATION=Quaternion(
# 		x=0.70833379,
# 		y=0.70586246,
# 		z=-0.00452118,
# 		w=-0.00097029,
# 	)

POSITION_CONTROL_EE_ORIENTATION=Quaternion(
    x=0.56770986, y=-0.49954435, z=0.5701564, w=-0.32106492
)