from collections import OrderedDict
import numpy as np
from sawyer_control.envs.sawyer_env_base import SawyerEnvBaseImage
from sawyer_control.core.serializable import Serializable
from sawyer_control.core.eval_util import get_stat_in_paths, \
    create_stats_ordered_dict
from sawyer_control.core.multitask_env import MultitaskEnv

class SawyerXYPushingImgMultitaskEnv(SawyerEnvBaseImage, MultitaskEnv):

    def __init__(self,
                 desired = None,
                 randomize_goal_on_reset=False,
                 #z=0.3,
                 pause_on_reset=False,
                 **kwargs
                 ):
        #TODO: MODIFY TO USE SAWYERENVBASE
        Serializable.quick_init(self, locals())
        SawyerEnv.__init__(self, **kwargs)
        MultitaskEnv.__init__(self, **kwargs)
        self.desired = np.zeros((4))
        self.action_mode = 'position'
        self.pause_on_reset = pause_on_reset
        #set pushing safety box here
        #goals are push the object to a spot, then reach a different position
        self.pushing_box_high_obj = np.array([0.66, 0.1]) # for the same = 0.02265
        self.pushing_box_low_obj = np.array([0.45, -0.1])
        self.pushing_box_high_ee = np.array([0.76, 0.145])
        self.pushing_box_low_ee = np.array([0.45, -0.12])
        self.pushing_box_low = np.array([0.44, -0.12])
        self.pushing_box_high = np.array([0.76, 0.145])
        high = np.concatenate((self.pushing_box_high_obj, self.pushing_box_high_ee)) #make sure to concatenate with itself
        low = np.concatenate((self.pushing_box_low_obj, self.pushing_box_low_ee))
        self.goal_space = Box(low, high)
        self.img_observation = True
        self.z = 0.23128
        self.img_observation = True
        self.reset_position = np.array([0.4499366724, 0.02265634, 0.23128])
        #self.set_safety_box(ee_low=np.array([0.405, -0.13, 0.23128]), ee_high=np.array([0.92, 0.156, 0.23128]))


    @property
    def goal_dim(self):
        return 4 #xy for object position, xy for end effector position
    def convert_obs_to_goals(self, obs):
        return obs[:, 21:24] # TODO: check this!
    def sample_goals(self, batch_size):
        return np.vstack([self.goal_space.sample() for i in range(batch_size)])

    def set_goal(self, goal,eval=True):
        print('pause')
        input()
        if eval:
            print('setting goal')
            obj_goal = np.concatenate((goal[:2], [self.z]))
            ee_goal = np.concatenate((goal[2:4], [self.z]))
            MultitaskEnv.set_goal(self, goal)
            #PAUSE FOR INPUT:
            self.thresh=False
            self._joint_act(obj_goal-self._end_effector_pose()[:3])
            input()
            self._joint_act(ee_goal - self._end_effector_pose()[:3])
            print('setting ee')
            input()
            #PAUSE FOR INPUT
            #PLACE OBJECT AT GOAL POSITION
            #MOVES ROBOT ARM TO GOAL POSITION:

            self.desired = goal
            self.thresh = True


    def get_image(self):
        temp = self.request_image()
        #update the get image in server. get an 84 x 84 x 3
        #reshape to 84 x 84 x 3
        temp = np.array(temp)
        temp = temp.reshape(84, 84, 3)
        img = temp[:40, 12:63]
        img = cv2.resize(img, (0, 0), fx=1.64705882, fy=2.1)
        observation = img
        observation = img / 255.0
        observation = observation.transpose()
        observation = observation.flatten()
        return observation

    def reward(self):
        '''
        this is isnt used by the multitask env at all
        :return:
        '''
        current = self._end_effector_pose()[:3]
        # differences = self.desired - current
        # reward = self.reward_function(differences)
        return current

    def _set_observation_space(self):
        self._observation_space = Box(np.zeros((21168,)), 1.0*np.ones(21168,))

    def _randomize_desired_end_effector_pose(self):
        #self.desired = np.random.uniform(self.safety_box_lows, self.safety_box_highs, size=(1, 3))[0]
        assert False # Don't randomize goal inside reset! In MultitaskEnv this is done by set_goal
        high = self.ee_safety_box_high
        low = self.ee_safety_box_low
        dx = np.random.uniform(low[0], high[0])
        dy = np.random.uniform(low[1], high[1])
        dz = np.random.uniform(low[2], high[2])
        self.desired =  np.array([dx, dy, dz])

    def compute_her_reward_np(self, observation, action, next_observation, goal):
        '''
        this shouldn't be used either
        :param observation:
        :param action:
        :param next_observation:
        :param goal:
        :return:
        '''
        return -np.linalg.norm(next_observation[21:24] - goal)

    def reset(self, vae_reset=False):
        self.in_reset = True
        # self._safe_move_to_neutral()
        self.thresh = False
        self._joint_act(self.reset_position  - self._end_effector_pose()[:3])
        self.in_reset = False
        self.thresh = True
        if self.pause_on_reset:
            #PAUSE AND MOVE OBJECT TO ITS RESET POSITION
            print('in reset')
            input()
        if self.img_observation:
            observation = self.get_image()
        else:
            observation = self._get_observation()
        return observation

    def _joint_act(self, action):
        ee_pos = self._end_effector_pose()
        if self.relative_pos_control:
            target_ee_pos = (ee_pos[:3] + action)
        else:
            target_ee_pos = action
        target_ee_pos[2] = self.z
        target_ee_pos[:2] = np.clip(target_ee_pos[:2], self.pushing_box_low, self.pushing_box_high)
        target_ee_pos = np.concatenate((target_ee_pos, ee_pos[3:]))
        angles = self.request_ik_angles(target_ee_pos, self._joint_angles())
        self.send_angle_action(angles)

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
        ))
        return temp

    def step(self, action):
        self._act(action)
        observation = self._get_observation()
        reward = self.reward() * self.reward_magnitude
        done = False
        info = {}
        #info['cartesian_goal'] = self.desired.copy()
        #pos = np.array(observation[21:24])
        #info['distance_to_goal'] = np.linalg.norm(pos - self.desired)
        if self.img_observation:
            observation = self.get_image()
        return observation, reward, done, info


    def _get_statistics_from_paths(self, paths):
        # statistics = OrderedDict()
        # stat_prefix = 'Test'
        # #distances_from_target = np.array([path['distance_to_goal'] for path in paths]).flatten()
        # distances_from_target = []
        # final_position_distances = []
        # for path in paths:
        #     distances_from_target += [p['distance_to_goal'] for p in path['env_infos']]
        #     final_position_distances += [[p['distance_to_goal'] for p in path['env_infos']][-1]]
        # final_position_distances = np.array(final_position_distances)
        # distances_from_target = np.array(distances_from_target)
        # statistics.update(self._update_statistics_with_observation(
        #     distances_from_target,
        #     stat_prefix,
        #     'End Effector Distance from Target'
        # ))
        #
        # statistics.update(self._update_statistics_with_observation(
        #     final_position_distances,
        #     stat_prefix,
        #     'Final End Effector Distance from Target'
        # ))

        return OrderedDict()
