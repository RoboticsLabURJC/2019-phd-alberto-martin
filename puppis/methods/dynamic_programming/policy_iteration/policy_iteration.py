import gym
import numpy as np

def compute_value_function(gym_env, policy, gamma=1.0):
    value_table = np.zeros(gym_env.nS)
    threshold = 1e-10
    while True:
        updated_value_table = np.zeros(gym_env.nS)
        for state in range(gym_env.nS):
            action = policy[state]
            for trans_prob, next_state, reward_prob, _ in gym_env.env.P[state][action]:
                value_table[state] = sum([trans_prob * (reward_prob + gamma * updated_value_table[next_state])])

        if np.sum(np.fabs(updated_value_table - value_table)) <= threshold:
            break

    return value_table

def extract_policy(gym_env, value_table, gamma=1.0):
    policy = np.zeros(gym_env.observation_space.n)
    for state in range(gym_env.observation_space.n):
        Q_table = np.zeros(gym_env.action_space.n)
        for action in range(gym_env.action_space.n):

