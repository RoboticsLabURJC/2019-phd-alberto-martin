import gym
import json
import logging
import random
import numpy as np
from gym import spaces
from gym.utils import seeding
from gym_pyxis.envs.gazebo.f1 import F1, f1_utils

logger = logging.getLogger(__name__)


class F1FollowLineCameraEnv(gym.Env):

    metadata = {'render.modes': ['human']}

    ACTION_MEANING = {
        0: "FORWARD",
        1: "LEFT",
        2: "RIGHT"
    }

    def __init__(self):
        self.model_name = 'f1_renault'
        self.f1 = F1()
        self.action_space = spaces.Discrete(3)
        self.reward_range = (-np.inf, np.inf)
        self.np_random = 0
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(480, 640, 1), dtype=np.uint8)
        self.seed()
        self.checkpoints = {}

    def load_checkpoints(self, filepath):
        with open(filepath, 'r') as fd:
            self.checkpoints = json.load(fd)

    @staticmethod
    def _compute_reward(image):
        # TODO: it is a naive approach
        robot_position = f1_utils.get_robot_position_respect_road(image)

        if robot_position == 'center_road':
            return 2.0
        elif robot_position == 'in_road':
            return 0.5
        return -1.0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        self.f1.reset_simulation()
        self.f1.un_pause_physics()
        self.f1.send_velocity_command(0.0, 0.0)
        if len(self.checkpoints) > 0:
            new_pose = self.checkpoints[random.choice(list(self.checkpoints.keys()))]
            self.f1.set_pose(new_pose[0], new_pose[1], new_pose[2],
                             new_pose[3], new_pose[4], new_pose[5], new_pose[6],
                             self.model_name)
        image = self.f1.get_camera_data()
        self.f1.pause_physics()
        return image

    def step(self, action):

        if action not in F1FollowLineCameraEnv.ACTION_MEANING:
            raise ValueError("Action {} not found".format(action))

        self.f1.un_pause_physics()

        if F1FollowLineCameraEnv.get_action_meaning(action) == 'FORWARD':
            self.f1.send_velocity_command(0.4, 0.0)
        elif F1FollowLineCameraEnv.get_action_meaning(action) == 'LEFT':
            self.f1.send_velocity_command(0.05, 0.2)
        elif F1FollowLineCameraEnv.get_action_meaning(action) == 'RIGHT':
            self.f1.send_velocity_command(0.05, -0.2)

        image = self.f1.get_camera_data()

        self.f1.pause_physics()
        reward = F1FollowLineCameraEnv._compute_reward(image)
        done = False
        if reward == -1:
            done = True

        return image, reward, done, {}

    def close(self):
        self.f1.send_velocity_command(0.0, 0.0)
        self.f1.reset_simulation()

    def render(self, mode='human', close=False):
        raise NotImplementedError()

    @staticmethod
    def get_action_meaning(action):
        if action in F1FollowLineCameraEnv.ACTION_MEANING:
            return F1FollowLineCameraEnv.ACTION_MEANING[action]
        return 'Unknown'
