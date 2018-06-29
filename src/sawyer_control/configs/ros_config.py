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
# RESET_ANGLES = np.array([4.76642555,
#                          4.85106224,
#                          4.20980651,
#                          3.75817663,
#                          2.83807707,
#                          0.64924514,
#                          4.70658493
#                     ])
RESET_ANGLES = np.array(
    [-1.43250394,
     -1.42687404,
     -2.18905854,
     -2.50070024,
     2.85557127,
     0.5682295,
     4.70658493
     ]
)
RESET_DICT = dict(zip(JOINT_NAMES, RESET_ANGLES))
