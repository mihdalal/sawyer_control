#!/usr/bin/python3
from sawyer_control.envs.sawyer_door import SawyerDoorEnv

env = SawyerDoorEnv(
    action_mode='position',
    config_name='austri_config',
    reset_free=False,
	position_action_scale=0.01,
	max_speed=0.2,
	use_compliant_position_controller=True
)
env.reset()
