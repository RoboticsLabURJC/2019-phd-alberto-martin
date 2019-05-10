import gym
import gym_pyxis
import unittest


class TestGazeboEnvs(unittest.TestCase):

    def test_turtlebot3_followline_camera_env(self):
        env = gym.make('Turtlebot3FollowLineCameraEnv-v0')
        state, reward, done, _ = env.step(action=0)
        state = env.reset()
        env.close()
        a = 0


if __name__ == '__main__':
    TestGazeboEnvs().test_turtlebot3_followline_camera_env()