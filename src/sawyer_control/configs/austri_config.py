from sawyer_control.configs.base_config import *
import numpy as np
from gym.spaces import Box

TORQUE_SAFETY_BOX_LOWS = np.array([0.4, -0.25, 0.2])
TORQUE_SAFETY_BOX_HIGHS = np.array([0.7, 0.25, 0.7])
TORQUE_SAFETY_BOX = Box(TORQUE_SAFETY_BOX_LOWS, TORQUE_SAFETY_BOX_HIGHS, dtype=np.float32)

POSITION_SAFETY_BOX_LOWS = np.array([ 0.5, 0,  0.33])
# POSITION_SAFETY_BOX_HIGHS = np.array([0.7, 0.13, 0.315])
POSITION_SAFETY_BOX_HIGHS = np.array([0.68, 0.13, 0.45])
POSITION_SAFETY_BOX = Box(POSITION_SAFETY_BOX_LOWS, POSITION_SAFETY_BOX_HIGHS, dtype=np.float32)
