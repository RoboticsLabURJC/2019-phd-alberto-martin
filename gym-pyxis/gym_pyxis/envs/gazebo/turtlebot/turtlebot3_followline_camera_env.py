import cv2
import gym
import logging
import numpy as np
from gym import spaces
from gym.utils import seeding
from gym_pyxis.envs.gazebo.turtlebot import Turtlebot3, turtlebot_utils

logger = logging.getLogger(__name__)


class Turtlebot3FollowLineCameraEnv(gym.Env):


    metadata = {'render.modes': ['human']}

    def __init__(self):

        self.turtlebot = Turtlebot3()
        self.action_space = spaces.Discrete(3)
        self.reward_range = (-np.inf, np.inf)
        self.np_random = 0
        self.seed()

    @staticmethod
    def _compute_reward(image):
        image_region = turtlebot_utils.get_image_region(image)

        if image_region == 'safe_region':
            return 2.0
        elif image_region == 'unsafe_region':
            return 0.5

        return 0.0

    def _is_done(self):
        return False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        self.turtlebot.reset_simulation()
        self.turtlebot.unpause_physics()
        image = self.turtlebot.get_camera_data()
        self.turtlebot.pause_physics()
        return image

    def step(self, action):
        self.turtlebot.unpause_physics()

        if action == 0:
            # FORWARD
            self.turtlebot.send_velocity_command(0.2, 0.0)
        elif action == 1:
            # LEFT
            self.turtlebot.send_velocity_command(0.05, 0.2)
        elif action == 2:
            # RIGHT
            self.turtlebot.send_velocity_command(0.05, -0.2)

        image = self.turtlebot.get_camera_data()

        # cv2.imwrite('image.png', image)

        self.turtlebot.pause_physics()
        reward = Turtlebot3FollowLineCameraEnv._compute_reward(image)
        done = self._is_done()

        return image, reward, done, {}

    def close(self):
        self.turtlebot.send_velocity_command(0.0, 0.0)
        self.turtlebot.reset_simulation()
