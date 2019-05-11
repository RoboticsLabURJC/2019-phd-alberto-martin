import gym
import gym_pyxis
import unittest
import random
import numpy as np


class TestGazeboEnvs(unittest.TestCase):

    def test_turtlebot3_followline_camera_env(self):
        env = gym.make('Turtlebot3FollowLineCameraEnv-v0')
        _ = env.reset()
        total_reward = 0.0
        actions = np.arange(env.action_space.n)
        count = 0
        while True:
            state, reward, done, _ = env.step(random.choice(actions))
            total_reward += reward
            count += 1
            if done:
                break

            if count == 1000:
                break

        env.reset()
        env.close()
        print('Total reward: {}'.format(total_reward))


if __name__ == '__main__':
    TestGazeboEnvs().test_turtlebot3_followline_camera_env()