import cv2
import gym
import gym_pyxis
import unittest
import random
import numpy as np
from gym_pyxis.envs.gazebo.f1 import F1, f1_utils


class TestF1(unittest.TestCase):

    def test_f1_model(self):
        f1 = F1()
        f1.send_velocity_command(0.0, 0.0)
        f1.reset_simulation()
        f1.get_odometry()
        image = f1.get_camera_data()
        print(image.shape)
        f1.set_pose(100.0, 0, 0, 0, 0, 0, 0, model_name='f1_renault')
        f1.send_velocity_command(1.0, 0.0)
        f1.send_velocity_command(0.0, 0.0)
        cv2.imshow('f1_camera', image)
        cv2.waitKey(0)

    def test_f1_images(self):
        f1 = F1()
        img = f1.get_camera_data()
        f1_position = f1_utils.get_robot_position_respect_road(img, debug=True)
        cv2.waitKey(0)
        print(f1_position)

    def test_f1_followline_env(self):
        env = gym.make('F1FollowLineCameraEnv-v0')
        env.load_checkpoints('../tools/f1/adjusted_checkpoints.json')
        _ = env.reset()
        total_reward = 0.0
        actions = np.arange(env.action_space.n)
        while True:
            state, reward, done, _ = env.step(random.choice(actions))
            total_reward += reward
            if done:
                break

        env.reset()
        env.close()
        print('Total reward: {}'.format(total_reward))
