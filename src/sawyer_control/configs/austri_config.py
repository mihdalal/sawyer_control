from sawyer_control.configs.base_config import *
TORQUE_SAFETY_BOX_LOWS = np.array([0.3, -0.4, 0.2])
TORQUE_SAFETY_BOX_HIGHS = np.array([0.7, 0.4, 0.7])
TORQUE_SAFETY_BOX = Box(TORQUE_SAFETY_BOX_LOWS, TORQUE_SAFETY_BOX_HIGHS)