import gym
import logging
import numpy as np
from gym import spaces
from gym.utils import seeding
from gym_pyxis.envs.gazebo.turtlebot import Turtlebot3, turtlebot_utils

logger = logging.getLogger(__name__)


class Turtlebot3FollowLineCameraEnv(gym.Env):

    metadata = {'render.modes': ['human']}

    ACTION_MEANING = {
        0: "FORWARD",
        1: "LEFT",
        2: "RIGHT"
    }

    def __init__(self):

        self.turtlebot = Turtlebot3()
        self.action_space = spaces.Discrete(3)
        self.reward_range = (-np.inf, np.inf)
        self.np_random = 0
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(480, 640, 1), dtype=np.uint8)
        self.seed()
        # TODO: improve the way to compute if an episode is done
        self._steps_count = 0
        self._rewards = []

    @staticmethod
    def _compute_reward(image):
        # TODO: it is a naive approach
        robot_position = turtlebot_utils.get_robot_position_respect_road(image)

        if robot_position == 'center_road':
            return 2.0
        elif robot_position == 'in_road':
            return 0.5

        return -1.0

    def _is_done(self):
        # TODO: improve the way to compute if an episode is done
        if self._steps_count >= 500:
            percentile = 70
            reward_bound = np.percentile(self._rewards, percentile)
            reward_mean = float(np.mean(self._rewards))
            if reward_mean > reward_bound:
                self._steps_count = 0
                self._rewards = []
                return True

        return False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        self._steps_count = 0
        self._rewards = []
        self.turtlebot.reset_simulation()
        self.turtlebot.unpause_physics()
        image = self.turtlebot.get_camera_data()
        self.turtlebot.pause_physics()
        return image

    def step(self, action):

        if action not in Turtlebot3FollowLineCameraEnv.ACTION_MEANING:
            raise ValueError("Action {} not found".format(action))

        self.turtlebot.unpause_physics()

        if Turtlebot3FollowLineCameraEnv.get_action_meaning(action) == 'FORWARD':
            self.turtlebot.send_velocity_command(0.2, 0.0)
        elif Turtlebot3FollowLineCameraEnv.get_action_meaning(action) == 'LEFT':
            self.turtlebot.send_velocity_command(0.05, 0.2)
        elif Turtlebot3FollowLineCameraEnv.get_action_meaning(action) == 'RIGHT':
            self.turtlebot.send_velocity_command(0.05, -0.2)

        image = self.turtlebot.get_camera_data()
        # cv2.imwrite('image.png', image)

        self.turtlebot.pause_physics()
        reward = Turtlebot3FollowLineCameraEnv._compute_reward(image)
        self._steps_count += 1
        self._rewards.append(reward)
        done = self._is_done()

        return image, reward, done, {}

    def close(self):
        self.turtlebot.send_velocity_command(0.0, 0.0)
        self.turtlebot.reset_simulation()

    def render(self, mode='human', close=False):
        raise NotImplementedError()

    @staticmethod
    def get_action_meaning(action):
        if action in Turtlebot3FollowLineCameraEnv.ACTION_MEANING:
            return Turtlebot3FollowLineCameraEnv.ACTION_MEANING[action]
        return 'Unknown'
