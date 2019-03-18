import gym
import gym_pyxis
import time
from gym_pyxis.envs import specs

if __name__ == '__main__':
    print('This is a robotics-gym')
    specs.environment_specs.world = '/opt/ros/kinetic/share/stage/worlds/simple.cfg'
    env = gym.make('WorldPlayerStageEnv-v0')
    time.sleep(30)
    env.close()
