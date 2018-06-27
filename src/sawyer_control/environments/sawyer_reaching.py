from collections import OrderedDict
import numpy as np
from sawyer_control.sawyer_env_base import SawyerEnv
from rllab.spaces.box import Box
from sawyer_control.serializable import Serializable

class SawyerJointSpaceReachingEnv(SawyerEnv):
    def __init__(self,
                 desired = None,
                 randomize_goal_on_reset=False,
                 **kwargs
                 ):
        Serializable.quick_init(self, locals())
        if desired is None:
            self._randomize_desired_angles()
        else:
            self._state_goal = desired
        self._randomize_goal_on_reset = randomize_goal_on_reset
        super().__init__(**kwargs)

    def reward(self):
        current = self._joint_angles()
        differences = self.compute_angle_difference(current, self._state_goal)
        reward = self.reward_function(differences)
        return reward

    def _get_statistics_from_paths(self, paths):
        statistics = OrderedDict()
        stat_prefix = 'Test'
        angle_differences, distances_outside_box = self._extract_experiment_info(paths)
        statistics.update(self._update_statistics_with_observation(
            angle_differences,
            stat_prefix,
            'Difference from Desired Joint Angle'
        ))

        return statistics

    def _extract_experiment_info(self, paths):
        obsSets = [path["env_info"] for path in paths]
        angles = []
        desired_angles = []
        positions = []
        for obsSet in obsSets:
            for observation in obsSet:
                observation = observation['true_state']
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

        self._observation_space = Box(lows, highs)

    def _randomize_desired_angles(self):
        self._state_goal = np.random.rand(1, 7)[0] * 2 - 1

    def reset(self):
        self.in_reset = True
        self._safe_move_to_neutral()
        self.in_reset = False
        if self._randomize_goal_on_reset:
            self._randomize_desired_angles()
        return self._get_observation()

class SawyerXYZReachingEnv(SawyerEnv):
    def __init__(self,
                 desired = None,
                 randomize_goal_on_reset=False,
                 **kwargs
                 ):
        Serializable.quick_init(self, locals())
        super().__init__(**kwargs)
        if desired is None:
            self._randomize_desired_end_effector_pose()
        else:
            self._state_goal = desired
        self._randomize_goal_on_reset = randomize_goal_on_reset

    def reward(self):
        current = self._end_effector_pose()[:3]
        differences = self._state_goal - current
        reward = self.reward_function(differences)
        return reward

    def _set_observation_space(self):
        lows = np.hstack((
            JOINT_VALUE_LOW['position'],
            JOINT_VALUE_LOW['velocity'],
            JOINT_VALUE_LOW['torque'],
            END_EFFECTOR_VALUE_LOW['position'],
            END_EFFECTOR_VALUE_LOW['angle'],
            END_EFFECTOR_VALUE_LOW['position'],
        ))

        highs = np.hstack((
            JOINT_VALUE_HIGH['position'],
            JOINT_VALUE_HIGH['velocity'],
            JOINT_VALUE_HIGH['torque'],
            END_EFFECTOR_VALUE_HIGH['position'],
            END_EFFECTOR_VALUE_HIGH['angle'],
            END_EFFECTOR_VALUE_HIGH['position'],
        ))

        self._observation_space = Box(lows, highs)

    def _get_random_ee_pose(self, batch_size=1):
        if self.action_mode == 'position':
            reaching_box_lows = self.ee_safety_box_low
            reaching_box_highs = self.ee_safety_box_high
        else:
            reaching_box_lows = self.not_reset_safety_box_lows + np.ones(3) * .2
            reaching_box_highs = self.not_reset_safety_box_highs - np.ones(3) * .2
        return np.random.uniform(reaching_box_lows, reaching_box_highs, size=(batch_size, 3))

    def _randomize_desired_end_effector_pose(self):
        self._state_goal = self._get_random_ee_pose()[0]

    def reset(self):
        # print('DESIRED', self._state_goal, 'ACTUAL', self._end_effector_pose()[0:3])
        self.in_reset = True
        self._safe_move_to_neutral()
        self.in_reset = False
        if self._randomize_goal_on_reset:
            self._randomize_desired_end_effector_pose()
        return self._get_observation()

    def _get_statistics_from_paths(self, paths):
        statistics = OrderedDict()
        stat_prefix = 'Test'
        distances_from_target, final_position_distances, final_10_positions_distances = self._extract_experiment_info(paths)
        statistics.update(self._update_statistics_with_observation(
            distances_from_target,
            stat_prefix,
            'End Effector Distance from Target'
        ))

        statistics.update(self._update_statistics_with_observation(
            final_10_positions_distances,
            stat_prefix,
            'Last 10 Step End Effector Distance from Target'
        ))

        statistics.update(self._update_statistics_with_observation(
            final_position_distances,
            stat_prefix,
            'Final End Effector Distance from Target'
        ))

        return statistics

    def _extract_experiment_info(self, paths):
        obsSets = [path["env_infos"] for path in paths]
        positions = []
        desired_positions = []
        distances = []
        final_positions = []
        final_desired_positions = []
        final_10_positions = []
        final_10_desired_positions = []
        for obsSet in obsSets:
            for observation in obsSet:
                observation = observation["true_state"]
                pos = np.array(observation[21:24])
                des = np.array(observation[28:31])
                distances.append(np.linalg.norm(pos - des))
                positions.append(pos)
                desired_positions.append(des)
            final_10_positions = positions[len(positions)-10:]
            final_10_desired_positions = desired_positions[len(desired_positions)-10:]
            final_positions.append(positions[-1])
            final_desired_positions.append(desired_positions[-1])

        distances = np.array(distances)
        final_positions = np.array(final_positions)
        final_desired_positions = np.array(final_desired_positions)
        final_10_positions = np.array(final_10_positions)
        final_10_desired_positions = np.array(final_10_desired_positions)
        final_position_distances = np.linalg.norm(final_positions - final_desired_positions, axis=1)
        final_10_positions_distances = np.linalg.norm(final_10_positions - final_10_desired_positions, axis=1)
        return [distances, final_position_distances, final_10_positions_distances]
