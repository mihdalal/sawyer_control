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

POSITION_RESET_POS = np.array([0.6211303, 0.0516888, 0.3437545])
RESET_ANGLES = np.array(
    [0.15519531,  0.91280955, - 2.02724218,  1.66523147, - 2.41472554,  1.15144241, - 2.47620893]
)
RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))
POSITION_CONTROL_EE_ORIENTATION=Quaternion(
    x=0.72693193, y=-0.03049006, z=0.6855942, w=-0.02451418
)
JOINT_SPACE_IMPD_MAX_TORQUES = 0.5 * np.array([8, 7, 6, 5, 4, 3, 2])
JOINT_SPACE_IMPD_RATE = 1000