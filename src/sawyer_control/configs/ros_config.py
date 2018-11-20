from geometry_msgs.msg import Quaternion
import numpy as np
#JOINT_CONTROLLER_SETTINGS
JOINT_POSITION_SPEED = .1
JOINT_POSITION_TIMEOUT = .2

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

#outside door angles right above handle
# RESET_ANGLES = np.array(
#     [ 0.48191699,  0.16221973, -1.76657319,  1.58018553,  0.23432715, -0.99837106,
#   1.95938277]
# )

#outside door angles right above handle far
RESET_ANGLES = np.array(
    [0.57940334,  0.15646386, -1.76931643,  1.84144044,  0.09261718, -1.2038486,
     2.11029887]
)



#inside door angle
RESET_ANGLES = np.array(
    [0.49415722,  0.23444238, -1.70247066,  1.52038765,  0.30878517, - 0.92729592,
     1.8574121]
)



RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))

# POSITION_CONTROL_EE_ORIENTATION=Quaternion(
# 		x=0.70833379,
# 		y=0.70586246,
# 		z=-0.00452118,
# 		w=-0.00097029,
# 	)

POSITION_CONTROL_EE_ORIENTATION=Quaternion(
    x=-0.4500708, y=0.56084436, z=-0.46314317, w=0.518062
)