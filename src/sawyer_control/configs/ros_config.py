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
RESET_ANGLES = np.array(
    [-2.06738291e-03, - 1.17774510e+00, - 2.17089849e-03,  2.17807722e+00,
     - 4.32617177e-04,  5.66576183e-01,  3.31357336e+00]
)
RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))

POSITION_RESET_POS = np.array([ 0.55, -0.05,  0.25471401])

# POSITION_CONTROL_EE_ORIENTATION=Quaternion(
# 		x=0.70833379,
# 		y=0.70586246,
# 		z=-0.00452118,
# 		w=-0.00097029,
# 	)

RESET_ANGLES = np.array(
    [0.20223731,  0.85522556, - 2.12354875,  1.59621096, - 2.52368164,  1.01539946, -2.40745807]
)



RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))

# POSITION_CONTROL_EE_ORIENTATION=Quaternion(
# 		x=0.70833379,
# 		y=0.70586246,
# 		z=-0.00452118,
# 		w=-0.00097029,
# 	)

POSITION_CONTROL_EE_ORIENTATION=Quaternion(
    x=0.72693193, y=-0.03049006, z=0.6855942, w=-0.02451418
)

JOINT_SPACE_IMPD_MAX_TORQUES = 0.5 * np.array([8, 7, 6, 5, 4, 3, 2])
JOINT_SPACE_IMPD_RATE = 1000