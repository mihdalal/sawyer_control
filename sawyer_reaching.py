from collections import OrderedDict
import numpy as np
from railrl.envs.ros.sawyer_env_base import SawyerEnv
from rllab.spaces.box import Box

JOINT_ANGLES_HIGH = np.array([
    1.70167993,
    1.04700017,
    3.0541791,
    2.61797006,
    3.05900002,
    2.09400001,
    3.05899961
])

JOINT_ANGLES_LOW = np.array([
    -1.70167995,
    -2.14700025,
    -3.0541801,
    -0.04995198,
    -3.05900015,
    -1.5708003,
    -3.05899989
])

JOINT_VEL_HIGH = 2*np.ones(7)
JOINT_VEL_LOW = -2*np.ones(7)

MAX_TORQUES = 0.5 * np.array([8, 7, 6, 5, 4, 3, 2])
JOINT_TORQUE_HIGH = MAX_TORQUES
JOINT_TORQUE_LOW = -1*MAX_TORQUES

JOINT_VALUE_HIGH = {
    'position': JOINT_ANGLES_HIGH,
    'velocity': JOINT_VEL_HIGH,
    'torque': JOINT_TORQUE_HIGH,
}
JOINT_VALUE_LOW = {
    'position': JOINT_ANGLES_LOW,
    'velocity': JOINT_VEL_LOW,
    'torque': JOINT_TORQUE_LOW,
}

END_EFFECTOR_POS_LOW = [
    0.3404830862298487,
    -1.2633121086809487,
    -0.5698485041484043
]

END_EFFECTOR_POS_HIGH = [
    1.1163239572333106,
    0.003933425621414761,
    0.795699462010194
]

END_EFFECTOR_ANGLE_LOW = -1*np.ones(4)
END_EFFECTOR_ANGLE_HIGH = np.ones(4)

END_EFFECTOR_VALUE_LOW = {
    'position': END_EFFECTOR_POS_LOW,
    'angle': END_EFFECTOR_ANGLE_LOW,
}

END_EFFECTOR_VALUE_HIGH = {
    'position': END_EFFECTOR_POS_HIGH,
    'angle': END_EFFECTOR_ANGLE_HIGH,
}
class SawyerJointSpaceReachingEnv(SawyerEnv):
    def __init__(self,
                 randomize_goal_on_reset=False,
                 **kwargs
                 ):
        self.randomize_goal_on_reset = randomize_goal_on_reset
        super().__init__(**kwargs)

    def reward(self, action):
        current = self._joint_angles()
        differences = self.compute_angle_difference(current, self.desired)
        reward = self.reward_function(differences)
        return reward

    def log_diagnostics(self, paths, logger=None):
        if logger==None:
            super().log_diagnostics(paths, logger=logger)
        else:
            statistics = OrderedDict()
            stat_prefix = 'Test'
            angle_differences, distances_outside_box = self._extract_experiment_info(paths)
            statistics.update(self._statistics_from_observations(
                angle_differences,
                stat_prefix,
                'Difference from Desired Joint Angle'
            ))

            if self.safety_box:
                statistics.update(self._statistics_from_observations(
                    distances_outside_box,
                    stat_prefix,
                    'End Effector Distance Outside Box'
                ))
            for key, value in statistics.items():
                logger.record_tabular(key, value)

    def _extract_experiment_info(self, paths):
        obsSets = [path["observations"] for path in paths]
        angles = []
        desired_angles = []
        positions = []
        for obsSet in obsSets:
            for observation in obsSet:
                angles.append(observation[:7])
                desired_angles.append(observation[24:31])
                positions.append(observation[21:24])

        angles = np.array(angles)
        desired_angles = np.array(desired_angles)

        differences = np.array([self.compute_angle_difference(angle_obs, desired_angle_obs)
                                for angle_obs, desired_angle_obs in zip(angles, desired_angles)])
        angle_differences = np.mean(differences, axis=1)
        distances_outside_box = np.array([self._compute_joint_distance_outside_box(pose) for pose in positions])
        return [angle_differences, distances_outside_box]

    def _set_observation_space(self):
        lows = np.hstack((
            JOINT_VALUE_LOW['position'],
            JOINT_VALUE_LOW['velocity'],
            JOINT_VALUE_LOW['torque'],
            END_EFFECTOR_VALUE_LOW['position'],
            END_EFFECTOR_VALUE_LOW['angle'],
            JOINT_VALUE_LOW['position'],
        ))

        highs = np.hstack((
            JOINT_VALUE_HIGH['position'],
            JOINT_VALUE_HIGH['velocity'],
            JOINT_VALUE_HIGH['torque'],
            END_EFFECTOR_VALUE_HIGH['position'],
            END_EFFECTOR_VALUE_HIGH['angle'],
            JOINT_VALUE_HIGH['position'],
        ))

        if self.randomize_goal_on_reset:
            self._randomize_desired_angles()
        else:
            self.desired = np.ones(7)
        self._observation_space = Box(lows, highs)

    def _randomize_desired_angles(self):
        self.desired = np.random.rand(1, 7)[0] * 2 - 1

    def reset(self):
        """
        Resets the state of the environment, returning an initial observation.
        Outputs
        -------
        observation : the initial observation of the space. (Initial reward is assumed to be 0.)
        """
        self.in_reset = True
        self.previous_angles = self._joint_angles()
        self._safe_move_to_neutral()
        self.previous_angles = self._joint_angles()
        self.in_reset = False
        if self.randomize_goal_on_reset:
            self._randomize_desired_angles()
        return self._get_observation()

class SawyerXYZReachingEnv(SawyerEnv):
    def __init__(self,
                 randomize_goal_on_reset=False,
                 **kwargs
                 ):
        self.randomize_goal_on_reset = randomize_goal_on_reset
        super().__init__(self, **kwargs)

    def reward(self, action):
        #TODO: IMPLEMENT
        pass

    def log_diagnostics(self, paths, logger=None):
        if logger==None:
            super().log_diagnostics(paths, logger=logger)
        else:
            #TODO: IMPLEMENT
            pass

    def _extract_experiment_info(self, paths):
        #TODO: IMPLEMENT
        pass

    def _set_observation_space(self):
        lows = np.hstack((
            JOINT_VALUE_LOW['position'],
            JOINT_VALUE_LOW['velocity'],
            JOINT_VALUE_LOW['torque'],
            END_EFFECTOR_VALUE_LOW['position'],
            JOINT_VALUE_LOW['position'],
        ))

        highs = np.hstack((
            JOINT_VALUE_HIGH['position'],
            JOINT_VALUE_HIGH['velocity'],
            JOINT_VALUE_HIGH['torque'],
            END_EFFECTOR_VALUE_HIGH['position'],
            JOINT_VALUE_HIGH['position'],
        ))

        if self.randomize_goal_on_reset:
            self._randomize_desired_end_effector_pose()
        else:
            self.desired = np.ones(3)
        self._observation_space = Box(lows, highs)

    def _randomize_desired_end_effector_pose(self):
        #TODO: IMPLEMENT
        raise NotImplementedError

    def reset(self):
        """
        Resets the state of the environment, returning an initial observation.
        Outputs
        -------
        observation : the initial observation of the space. (Initial reward is assumed to be 0.)
        """
        self.in_reset = True
        self.previous_angles = self._joint_angles()
        self._safe_move_to_neutral()
        self.previous_angles = self._joint_angles()
        self.in_reset = False
        if self.randomize_goal_on_reset:
            self._randomize_desired_end_effector_pose()
        return self._get_observation()