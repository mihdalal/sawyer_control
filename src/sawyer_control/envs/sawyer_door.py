from collections import OrderedDict
from gym.spaces import Dict
import numpy as np
import rospy
from gym.spaces import Box
from sawyer_control.envs.sawyer_env_base import SawyerEnvBase
from sawyer_control.core.serializable import Serializable
from sawyer_control.core.eval_util import get_stat_in_paths, \
	create_stats_ordered_dict
from sawyer_control.pd_controllers.joint_angle_pd_controller import AnglePDController
from sawyer_control.core.serializable import Serializable
from sawyer_control.core.multitask_env import MultitaskEnv
from sawyer_control.configs.config import config_dict as config
from sawyer_control.srv import observation
from sawyer_control.srv import getRobotPoseAndJacobian
from sawyer_control.srv import ik
from sawyer_control.srv import angle_action
from sawyer_control.srv import image
from sawyer_control.msg import actions
import cv2
import time
class SawyerDoorXYZEnv(SawyerEnvBase):
	def __init__(self,
				 fixed_goal=(1, 1, 1),
				 indicator_threshold=.05,
				 reward_type='hand_distance',
				 normalize=True,
				 transpose=True,
				 **kwargs
				 ):
		Serializable.quick_init(self, locals())
		SawyerEnvBase.__init__(self, **kwargs)
		if self.action_mode == 'torque':
			self.goal_space = self.config.TORQUE_SAFETY_BOX
		else:
			self.goal_space = self.config.POSITION_SAFETY_BOX
		self.indicator_threshold = indicator_threshold
		self.reward_type = reward_type
		self._state_goal = np.array(fixed_goal)
		self.observation_space = Dict([
			('observation', self.obs_space),
			('desired_goal', self.obs_space),
			('achieved_goal', self.obs_space),
			('state_observation', self.obs_space),
			('state_desired_goal', self.obs_space),
			('state_achieved_goal', self.obs_space), ])


		# self.reset_angles = np.array([0.5570849609375, -0.136208984375, -1.241103515625, 1.2219013671875,
		# 							  0.0895439453125, -0.7405, 1.4465234375])
		# self.reset_angles_1 = np.array([0.5679873046875, -0.788544921875, -0.6379423828125,
		# 								1.70463671875, 0.0512109375, -0.9890234375, 0.5703583984375])
		# real angles:
		self.reset_angles= np.array([0.7231025390625, -0.019921875, -1.5107236328125,
									 1.286544921875, 0.2815166015625, -0.6222880859375, 1.614041015625])
		self.do_reset = False
		self.transpose = transpose
		self.normalize = normalize
		self.prev_time = time.time()
		self.num_resets = 0


	def _set_observation_space(self):
		lows = np.array(self.config.END_EFFECTOR_VALUE_LOW['position'])
		highs = np.array(self.config.END_EFFECTOR_VALUE_HIGH['position'])
		self.obs_space = Box( lows, highs)

	def reset(self):
		self.num_resets += 1
		if self.num_resets > 0:
			self.do_reset = True
			if self.action_mode == "joint_space_impd" or self.action_mode == "position":
				self._position_act(self.reset_pos - self._get_endeffector_pose())
			else:
				self._reset_robot()
			#self._state_goal = self.sample_goal()
			# self.in_reset = False
			self.do_reset = False
		return self._get_obs()

	def _get_info(self):
		#hand_distance = np.linalg.norm(self._state_goal - self._get_endeffector_pose())
		hand_distance = 0
		return dict(
			hand_distance=hand_distance,
			hand_success=0
			# hand_success=(hand_distance < self.indicator_threshold).astype(float)
		)

	def get_diagnostics(self, paths, prefix=''):
		statistics = OrderedDict()
		for stat_name in [
			'hand_distance',
			'hand_success',
		]:
			stat_name = stat_name
			stat = get_stat_in_paths(paths, 'env_infos', stat_name)
			statistics.update(create_stats_ordered_dict(
				'%s%s' % (prefix, stat_name),
				stat,
				always_show_all_stats=True,
			))
			statistics.update(create_stats_ordered_dict(
				'Final %s%s' % (prefix, stat_name),
				[s[-1] for s in stat],
				always_show_all_stats=True,
			))
		return statistics

	"""
	Multitask functions
	"""

	def _get_obs(self):
		ee_pos = self._get_endeffector_pose()
		# state_obs = super()._get_obs()
		return dict(
			observation=ee_pos,
			desired_goal=self._state_goal,
			achieved_goal=ee_pos,
			state_observation=ee_pos,
			state_desired_goal=self._state_goal,
			state_achieved_goal=ee_pos,
		)

	def get_goal(self):
		return {}

	def send_angle_action(self, action, target, in_reset=False):
		self.request_angle_action(action, target, in_reset=in_reset)
	def request_angle_action(self, angles, pos, in_reset=False):
		dist = np.linalg.norm(self._get_endeffector_pose() - pos)
		duration = dist / self.max_speed
		rospy.wait_for_service('angle_action')
		if self.do_reset:
			try:
				execute_action = rospy.ServiceProxy('angle_action', angle_action, persistent=True)
				impd = self.action_mode == "position"
				execute_action(self.reset_angles, impd, duration, self.do_reset)
				return None
			except rospy.ServiceException as e:
				h = 1
		else:
			try:
				execute_action = rospy.ServiceProxy('angle_action', angle_action, persistent=True)
				impd = self.action_mode == "position"
				execute_action(angles, impd, duration, self.do_reset)
				return None
			except rospy.ServiceException as e:
				h = 1

	def get_image_from_png(self, image, width=84, height=84):
		image = self.request_image()
		if image is None:
			raise Exception('Unable to get image from image server')
		image = np.asarray(image).reshape(84, 84, 3)
		import scipy.misc
		image = scipy.misc.imresize(
			image,
			(width, height, 3)
		)
		return image

	def get_img_goal(self, i):

		img = cv2.imread('/home/shikharbahl/ros_ws/src/sawyer_control/src/sawyer_control/examples/door_images/' + str(i) + '.png')
		return img
	def sample_goals(self, batch_size):
		if self.fix_goal:
			goals = np.repeat(
				self.fixed_goal.copy()[None],
				batch_size,
				0
			)
		else:
			indices = np.random.randint(0, 5000, batch_size)
			goals = np.array([self.get_img_goal(i) for i in indices])
			# goal_dict = {
			# 	'image_desired_goal': goals,
			# 	'desired_goal': goals}
			# return goal_dict
		return goals
	def identity(self):
		return 1
	def set_presampled_goals(self, unused):
		return None

	def sample_goals_multi(self, batch_size):
		if self.fix_goal:
			goals = np.repeat(
				self.fixed_goal.copy()[None],
				batch_size,
				0
			)
		else:
			goals = np.random.uniform(
				self.goal_space.low,
				self.goal_space.high,
				size=(batch_size, self.goal_space.low.size),
			)
		return goals

	def compute_rewards(self, actions, obs):
		# achieved_goals = obs['state_achieved_goal']
		# desired_goals = obs['state_desired_goal']
		# hand_pos = achieved_goals
		# goals = desired_goals
		# distances = np.linalg.norm(hand_pos - goals, axis=1)
		# if self.reward_type == 'hand_distance':
		#     r = -distances
		# elif self.reward_type == 'hand_success':
		#     r = -(distances < self.indicator_threshold).astype(float)
		# else:
		#     raise NotImplementedError("Invalid/no reward type.")
		return np.array([0])

	@property
	def goal_dim(self):
		return 3
	def _get_obs(self):
		image_obs = self.get_image()
		if self.normalize:
			image_obs = image_obs / 255.0
		if self.transpose:
			image_obs = image_obs.transpose()
		return dict(
					image_observation=image_obs.flatten()
		)

	def convert_obs_to_goals(self, obs):
		return obs['state_desired_goal']

	def step(self, action):

		self._act(action)
		observation = self._get_obs()
		reward = 0
		info = self._get_info()
		done = False
		return observation, reward, done, info
	def set_to_goal(self, goal):

		# self._state_goal = goal
		# if self.action_mode == 'position':
		# 	self._position_act(goal - self._get_endeffector_pose()[:3])
		return self.sample_goals(1)[0]
	def get_diagnostics(self, paths, prefix=''):
		statistics = OrderedDict()
		for stat_name in [
			'hand_distance',
			'hand_success',
		]:
			stat_name = stat_name
			stat = get_stat_in_paths(paths, 'env_infos', stat_name)
			statistics.update(create_stats_ordered_dict(
				'%s%s' % (prefix, stat_name),
				stat,
				always_show_all_stats=True,
			))
			statistics.update(create_stats_ordered_dict(
				'Final %s%s' % (prefix, stat_name),
				[s[-1] for s in stat],
				always_show_all_stats=True,
			))
		return statistics
