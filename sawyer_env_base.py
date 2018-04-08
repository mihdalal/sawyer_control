import math
import time
from collections import OrderedDict
import numpy as np
import rospy
from experiments.murtaza.ros.Sawyer.pd_controller import PDController
from railrl.misc.eval_util import create_stats_ordered_dict
from serializable import Serializable
from rllab.spaces.box import Box
from sawyer_control.srv import observation
from sawyer_control.msg import actions
from sawyer_control.srv import getRobotPoseAndJacobian
from rllab.envs.base import Env

class SawyerEnv(Env, Serializable):
    def __init__(
            self,
            update_hz=20,
            safety_box=True,
            reward='huber',
            huber_delta=10,
            safety_force_magnitude=2,
            safety_force_temp=1.05,
            safe_reset_length=150,
            reward_magnitude=1,
            use_safety_checks=False,
    ):
        Serializable.quick_init(self, locals())
        self.init_rospy(update_hz)
        
        self.box_lows = np.array([-0.5888, -.6704, .04259])
        self.box_highs = np.array([.7506, 0.87129, .9755])
        self.joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

        self.arm_name = 'right'
        self.use_safety_checks = use_safety_checks
        self.reward_magnitude = reward_magnitude
        self.safety_box = safety_box
        self.safe_reset_length = safe_reset_length
        self.huber_delta = huber_delta
        self.safety_force_magnitude = safety_force_magnitude
        self.safety_force_temp = safety_force_temp
        self.PDController = PDController()

        max_torques = 0.5 * np.array([8, 7, 6, 5, 4, 3, 2])
        joint_torque_high = max_torques
        joint_torque_low = -1 * max_torques

        self._action_space = Box(
            joint_torque_low,
            joint_torque_high
        )

        if reward == 'MSE':
            self.reward_function = self._MSE_reward
        elif reward == 'huber':
            self.reward_function = self._Huber_reward
        else:
            self.reward_function = self._Norm_reward

        self._set_observation_space()
        self.get_latest_pose_jacobian_dict()
        self.in_reset = True
        self.amplify = np.ones(1) #by default, no amplifications

    def _act(self, action):
        if self.safety_box:
            self.get_latest_pose_jacobian_dict()
            truncated_dict = self.check_joints_in_box()
            if len(truncated_dict) > 0:
                forces_dict = self._get_adjustment_forces_per_joint_dict(truncated_dict)
                torques = np.zeros(7)
                for joint in forces_dict:
                    torques = torques + np.dot(truncated_dict[joint][1].T, forces_dict[joint]).T
                action = action + torques
        if self.in_reset:
            np.clip(action, -4, 4, out=action)
        if not self.in_reset:
            action = self.amplify * action
            action = np.clip(np.asarray(action),-MAX_TORQUES, MAX_TORQUES)

        self.send_action(action)
        self.rate.sleep()
        return action

    def _reset_within_threshold(self):
        desired_neutral = self.PDController._des_angles
        # note PDController.des_angles is a map between joint name to angle while joint_angles is a list of angles
        desired_neutral = np.array([desired_neutral[joint] for joint in self.joint_names])
        actual_neutral = (self._joint_angles())
        errors = self.compute_angle_difference(desired_neutral, actual_neutral)
        ERROR_THRESHOLD = .15*np.ones(7)
        is_within_threshold = (errors < ERROR_THRESHOLD).all()
        return is_within_threshold

    def _wrap_angles(self, angles):
        return angles % (2*np.pi)

    def _joint_angles(self):
        angles, _, _, _ = self.request_observation()
        angles = np.array(angles)
        return angles

    def _end_effector_pose(self):
        _, _, _, endpoint_pose = self.request_observation()
        return np.array(endpoint_pose)

    def _MSE_reward(self, differences):
        reward = -np.mean(differences**2)
        return reward

    def _Huber_reward(self, differences):
        a = np.abs(np.mean(differences))
        if a <= self.huber_delta:
            reward = -1 / 2 * a ** 2 * self.reward_magnitude
        else:
            reward = -1 * self.huber_delta * (a - 1 / 2 * self.huber_delta) * self.reward_magnitude
        return reward

    def _Norm_reward(self, differences):
        return np.linalg.norm(differences)

    def compute_angle_difference(self, angles1, angles2):
        self._wrap_angles(angles1)
        self._wrap_angles(angles2)
        deltas = np.abs(angles1 - angles2)
        differences = np.minimum(2 * np.pi - deltas, deltas)
        return differences

    def step(self, action):
        self.nan_check(action)
        actual_commanded_action = self._act(action)
        observation = self._get_observation()
        reward = self.reward(action)

        if self.use_safety_checks:
            out_of_box = self.safety_box_check()
            high_torque = self.high_torque_check(actual_commanded_action)
            unexpected_velocity = self.unexpected_velocity_check()
            unexpected_torque = self.unexpected_torque_check()
            done = out_of_box or high_torque or unexpected_velocity or unexpected_torque
        else:
            done = False
        info = {}
        return observation, reward, done, info

    def reward(self, action):
        raise NotImplementedError

    def safety_box_check(self):
        # TODO: tune this check
        self.get_latest_pose_jacobian_dict()
        truncated_dict = self.check_joints_in_box()
        terminate_episode = False
        if len(truncated_dict) > 0:
            for joint in truncated_dict.keys():
                dist = self._compute_joint_distance_outside_box(truncated_dict[joint][0])
                if dist > .19:
                    if not self.in_reset:
                        print('safety box failure during train/eval: ', joint, dist)
                        terminate_episode = True
                    else:
                        raise EnvironmentError('safety box failure during reset: ', joint, dist)
        return terminate_episode

    def jacobian_check(self):
        ee_jac = self.pose_jacobian_dict['right_hand'](1)
        if np.linalg.det(ee_jac) == 0:
            self._act(self._randomize_desired_angles())

    def unexpected_torque_check(self):
        #TODO: redesign this check
        #we care about the torque that was observed to make sure it hasn't gone too high
        new_torques = self.get_observed_torques_minus_gravity()
        if not self.in_reset:
            ERROR_THRESHOLD = np.array([25, 25, 25, 25, 666, 666, 10])
            is_peaks = (np.abs(new_torques) > ERROR_THRESHOLD).any()
            if is_peaks:
                print('unexpected_torque during train/eval: ', new_torques)
                return True
        else:
            ERROR_THRESHOLD = np.array([25, 25, 25, 30, 666, 666, 10])
            is_peaks = (np.abs(new_torques) > ERROR_THRESHOLD).any()
            if is_peaks:
                raise EnvironmentError('unexpected torques during reset: ', new_torques)
        return False

    def unexpected_velocity_check(self):
        #TODO: tune this check
        _, velocities, _, _ = self.request_observation()
        velocities = np.array(velocities)
        ERROR_THRESHOLD = 5 * np.ones(7)
        is_peaks = (np.abs(velocities) > ERROR_THRESHOLD).any()
        if is_peaks:
            print('unexpected_velocities during train/eval: ', velocities)
            if not self.in_reset:
                return True
            else:
                raise EnvironmentError('unexpected velocities during reset: ', velocities)
        return False

    def high_torque_check(self, commanded_torques):
        # TODO: tune this check
        new_torques = np.abs(commanded_torques)
        current_angles = self._joint_angles()
        position_deltas = np.abs(current_angles - self.previous_angles)
        DELTA_THRESHOLD = .05 * np.ones(7)
        ERROR_THRESHOLD = [11, 15, 15, 15, 666, 666, 10]
        violation = False
        for i in range(len(new_torques)):
            if new_torques[i] > ERROR_THRESHOLD[i] and position_deltas[i] < DELTA_THRESHOLD[i]:
                violation=True
                print("violating joint:", i)
        if violation:
            print('high_torque:', new_torques)
            print('positions', position_deltas)
            if not self.in_reset:
                return True
            else:
                raise EnvironmentError('ERROR: Applying large torques and not moving')
        self.previous_angles = current_angles
        return False

    def nan_check(self, action):
        for val in action:
            if math.isnan(val):
                raise EnvironmentError('ERROR: NaN action attempted')

    def _get_observation(self):
        angles = self._joint_angles()
        _, velocities, torques, _ = self.request_observation()
        velocities = np.array(velocities)
        torques = np.array(torques)
        endpoint_pose = self._end_effector_pose()

        temp = np.hstack((
            angles,
            velocities,
            torques,
            endpoint_pose,
            self.desired
        ))
        return temp

    def _safe_move_to_neutral(self):
        for i in range(self.safe_reset_length):
            cur_pos, cur_vel, _, _ = self.request_observation()
            torques = self.PDController._compute_pd_forces(cur_pos, cur_vel)
            actual_commanded_actions = self._act(torques)
            curr_time = time.time()
            self.init_delay = curr_time
            if self.previous_angles_reset_check():
                break
            if self.use_safety_checks:
                self.safety_box_check()
                self.unexpected_torque_check()
                self.high_torque_check(actual_commanded_actions)
                self.unexpected_velocity_check()

    def previous_angles_reset_check(self):
        close_to_desired_reset_pos = self._reset_within_threshold()
        _, velocities, _, _ = self.request_observation()
        velocities = np.abs(np.array(velocities))
        VELOCITY_THRESHOLD = .002 * np.ones(7)
        no_velocity = (velocities < VELOCITY_THRESHOLD).all()
        return close_to_desired_reset_pos and no_velocity

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
        return self._get_observation()

    def get_latest_pose_jacobian_dict(self):
        self.pose_jacobian_dict = self._get_robot_pose_jacobian_client('right')

    def _get_robot_pose_jacobian_client(self, name):
        rospy.wait_for_service('get_robot_pose_jacobian')
        try:
            get_robot_pose_jacobian = rospy.ServiceProxy('get_robot_pose_jacobian', getRobotPoseAndJacobian,
                                                         persistent=True)
            resp = get_robot_pose_jacobian(name)
            pose_jac_dict = self.get_pose_jacobian_dict(resp.poses, resp.jacobians)
            return pose_jac_dict
        except rospy.ServiceException as e:
            print(e)

    def get_pose_jacobian_dict(self, poses, jacobians):
        pose_jacobian_dict = {}
        pose_counter = 0
        jac_counter = 0
        poses = np.array(poses)
        jacobians = np.array(jacobians)
        for joint in self.joint_names[2:]:
            pose = poses[pose_counter:pose_counter + 3]
            jacobian = np.array([
                jacobians[jac_counter:jac_counter + 7],
                jacobians[jac_counter + 7:jac_counter + 14],
                jacobians[jac_counter + 14:jac_counter + 21],
            ])
            pose_counter += 3
            jac_counter += 21
            pose_jacobian_dict[joint] = [pose, jacobian]
        return pose_jacobian_dict

    def _get_positions_from_pose_jacobian_dict(self):
        poses = []
        for joint in self.pose_jacobian_dict.keys():
            poses.append(self.pose_jacobian_dict[joint][0])
        return np.array(poses)

    def check_joints_in_box(self):
        joint_dict = self.pose_jacobian_dict.copy()
        keys_to_remove = []
        for joint in joint_dict.keys():
            if self._pose_in_box(joint_dict[joint][0]):
                keys_to_remove.append(joint)
        for key in keys_to_remove:
            del joint_dict[key]
        return joint_dict

    def _pose_in_box(self, pose):
        within_box = [curr_pose > lower_pose and curr_pose < higher_pose
                      for curr_pose, lower_pose, higher_pose
                      in zip(pose, self.box_lows, self.box_highs)]
        return all(within_box)

    def _get_adjustment_forces_per_joint_dict(self, joint_dict):
        forces_dict = {}
        for joint in joint_dict:
            force = self._get_adjustment_force_from_pose(joint_dict[joint][0])
            forces_dict[joint] = force
        return forces_dict

    def _get_adjustment_force_from_pose(self, pose):
        x, y, z = 0, 0, 0

        curr_x = pose[0]
        curr_y = pose[1]
        curr_z = pose[2]
        if curr_x > self.box_highs[0]:
            x = -1 * np.exp(np.abs(curr_x - self.box_highs[0]) * self.safety_force_temp) * self.safety_force_magnitude
        elif curr_x < self.box_lows[0]:
            x = np.exp(np.abs(curr_x - self.box_lows[0]) * self.safety_force_temp) * self.safety_force_magnitude

        if curr_y > self.box_highs[1]:
            y = -1 * np.exp(np.abs(curr_y - self.box_highs[1]) * self.safety_force_temp) * self.safety_force_magnitude
        elif curr_y < self.box_lows[1]:
            y = np.exp(np.abs(curr_y - self.box_lows[1]) * self.safety_force_temp) * self.safety_force_magnitude

        if curr_z > self.box_highs[2]:
            z = -1 * np.exp(np.abs(curr_z - self.box_highs[2]) * self.safety_force_temp) * self.safety_force_magnitude
        elif curr_z < self.box_lows[2]:
            z = np.exp(np.abs(curr_z - self.box_highs[2]) * self.safety_force_temp) * self.safety_force_magnitude
        return np.array([x, y, z])

    def _compute_joint_distance_outside_box(self, pose):
        curr_x = pose[0]
        curr_y = pose[1]
        curr_z = pose[2]
        if(self._pose_in_box(pose)):
            x, y, z = 0, 0, 0
        else:
            x, y, z = 0, 0, 0
            if curr_x > self.box_highs[0]:
                x = np.abs(curr_x - self.box_highs[0])
            elif curr_x < self.box_lows[0]:
                x = np.abs(curr_x - self.box_lows[0])
            if curr_y > self.box_highs[1]:
                y = np.abs(curr_y - self.box_highs[1])
            elif curr_y < self.box_lows[1]:
                y = np.abs(curr_y - self.box_lows[1])
            if curr_z > self.box_highs[2]:
                z = np.abs(curr_z - self.box_highs[2])
            elif curr_z < self.box_lows[2]:
                z = np.abs(curr_z - self.box_lows[2])
        return np.linalg.norm([x, y, z])

    def log_diagnostics(self, paths, logger=None):
        if logger==None:
            pass
        else:
            raise NotImplementedError


    def _statistics_from_observations(self, observation, stat_prefix, log_title):
        statistics = OrderedDict()
        statistics.update(create_stats_ordered_dict(
            '{} {}'.format(stat_prefix, log_title),
            observation,
        ))
        return statistics


    @property
    def action_space(self):
        return self._action_space

    @property
    def observation_space(self):
        return self._observation_space

    def init_rospy(self, update_hz):
        rospy.init_node('sawyer_env', anonymous=True)
        self.action_publisher = rospy.Publisher('actions_publisher', actions, queue_size=10)
        self.rate = rospy.Rate(update_hz)

    def send_action(self, action):
        self.action_publisher.publish(action)

    def request_observation(self):
        rospy.wait_for_service('observations')
        try:
            request = rospy.ServiceProxy('observations', observation, persistent=True)
            obs = request()
            return (
                    obs.angles,
                    obs.velocities,
                    obs.torques,
                    obs.endpoint_pose
            )
        except rospy.ServiceException as e:
            print(e)

    @property
    def horizon(self):
        raise NotImplementedError

    def terminate(self):
        self.reset()

    def _set_observation_space(self):
        raise NotImplementedError
