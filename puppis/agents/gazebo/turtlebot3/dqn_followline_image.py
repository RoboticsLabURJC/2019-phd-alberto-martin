import gym
import time
import gym_pyxis
import random
import numpy as np

from .utils import wrappers

if __name__ == "__main__":
    env = wrappers.make_env('Turtlebot3FollowLineCameraEnv-v0')
    state = env.reset()
    total_reward = 0.0
    actions = np.arange(env.action_space.n)
    count = 0
    while True:
        start_ts = time.time()
        state, reward, done, _ = env.step(random.choice(actions))
        total_reward += reward
        count += 1
        if done:
            break

        if count == 1000:
            env.reset()

    print('total reward: {}'.format(total_reward))