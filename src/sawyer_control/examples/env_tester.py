#!/usr/bin/python3
from sawyer_control.envs.sawyer_door import SawyerDoorEnv
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerDoorEnv(
    action_mode='position',
    config_name='austri_config',
    reset_free=False,
    use_state_based_door_angle=True,
	position_action_scale=0.05,
	max_speed=0.15,
	use_compliant_position_controller=True
)
# env = SawyerReachXYZEnv(
#     action_mode='position',
#     config_name='austri_config',
# )
# for i in range(1):
#     # print('RESET')
#     env.reset_motor_pos = env.dy.reset([1])[0]
#     print('Relative Motor Position',env._get_relative_motor_pos())

print(env._get_joint_angles())
print(env._get_obs()[14:])
