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
[-0.64384375, -1.2390146484375, 0.4955966796875, 1.96015234375, -0.47374609375, 0.9081484375, 0.67590625]
# [-0.674080078125, -1.43962890625, 0.4098740234375, 2.246068359375, -0.38748828125, 0.757271484375, 0.569533203125]
    # [-2.06738291e-03, - 1.17774510e+00, - 2.17089849e-03,  2.17807722e+00,
    #  - 4.32617177e-04,  5.66576183e-01,  3.31357336e+00]
)
RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))

POSITION_CONTROL_EE_ORIENTATION=Quaternion(
		x=0.70833379,
		y=0.70586246,
		z=-0.00452118,
		w=-0.00097029,
	)