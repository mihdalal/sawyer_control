#!/usr/bin/python3
from sawyer_control.envs.sawyer_door import SawyerDoorEnv
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerDoorEnv(
    action_mode='position',
    config_name='austri_config',
    reset_free=False,
    use_state_based_door_angle=True
)
# env = SawyerReachXYZEnv(
#     action_mode='position',
#     config_name='austri_config',
# )
env.reset()
print(env._get_joint_angles())
print(env._get_obs()[14:])