from geometry_msgs.msg import Quaternion
import numpy as np
#JOINT_CONTROLLER_SETTINGS
JOINT_POSITION_SPEED = .15
JOINT_POSITION_TIMEOUT = .15

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

#inside door angle
RESET_ANGLES = np.array(
    [0.57168555, -0.10354199, -1.68741989,  1.91788673,  1.40054691,  1.71397173, -2.56532025]
)



RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))

# POSITION_CONTROL_EE_ORIENTATION=Quaternion(
# 		x=0.70833379,
# 		y=0.70586246,
# 		z=-0.00452118,
# 		w=-0.00097029,
# 	)

POSITION_CONTROL_EE_ORIENTATION=Quaternion(
    x=0.99907899, y=-0.03476779, z=-0.02298597, w=0.01019814
)
