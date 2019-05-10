import os
import gym
import gym_pyxis
import time
from gym_pyxis.envs import specs

if __name__ == '__main__':

    env = gym.make('SimpleRoomsEnv-v0')
    # env = gym.make('CliffWalkingEnv-v0')
    # env = gym.make('WindyGridworldEnv-v0')
    state = env.reset()

    cont = 0
    while cont != 100:
        cont += 1
        env._render()

    env.close()
