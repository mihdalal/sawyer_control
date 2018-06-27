import numpy as np
joint_names = ['right_j0',
               'right_j1',
               'right_j2',
               'right_j3',
               'right_j4',
               'right_j5',
               'right_j6'
               ]
reset_angles = np.ones(7) #TODO: SET THIS TO A REASONABLE POSITION 
reset_dict = dict(zip(joint_names, reset_angles))