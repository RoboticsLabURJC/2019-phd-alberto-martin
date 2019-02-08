---
layout: default
---
# Reinforcement Learning State of the Art
[Sutton & Barto Book](http://incompleteideas.net/book/bookdraft2017nov5.pdf)
1. **Tabular Solutions Methods**: the methods can often find exact solutions, that is, they can often find exactly the optimal value function and the optimal policy.

 * **Multi-armed Bandits**: special case with only a single state

 * **Finite Markov Decision Processes**: general problem formulation for Reinforcement Learning problems.

 * **Dynamic Programming**: method to solve FMDP. Needs a model of environment.

 * **Monte Carlo Methods**: method to solve FMDP. Model not required, conceptually simple but are not well suited for step-by-step incremental computation.

 * **Temporal-Difference Learning**: method to solve FMDP. Model not required and are fully incremental, but are more complex to analyze.

2. **Approximate Solution Methods**


## Multi-armed Bandits
[//]: https://www.analyticsvidhya.com/blog/2018/09/reinforcement-multi-armed-bandit-scratch-python/

A one-armed bandit is a simple slot machine wherein you insert a coin into the machine, pull a lever, and get an immediate reward. A multi-armed bandit is a complicated slot machine wherein instead of 1, there are several levers which a gambler can pull, with each lever giving a different return.  The probability distribution for the reward corresponding to each lever is different and is unknown to the gambler.

![Multi-armed Bandits](multi_armed_bandit_problem.png "Multi-arned Bandits")

The task is to identify which lever to pull in order to get maximum reward after a given a set of trials. This problem statement is like a single Markov decision process. Each arm chosen is equivalent to an action, which then leads to an immediate reward.

### Exploration Exploitation
The below table shows the sample of results for a 5-armed bandit with arms labelled as 1,2,3,4 and 5:

| Arm    | 1 | 2 | 3 | 4 | 5 | 3 | 3 | 2 | 1 | 4 | 2 |
|--------|---|---|---|---|---|---|---|---|---|---|---|
| Reward | 0 | 0 | 1 | 1 | 0 | 1 | 1 | 0 | 1 | 0 | 0 |

This is called [Bernoulli](https://en.wikipedia.org/wiki/Bernoulli_distribution), as the reward returned is either 1 or 0. In this example, it looks like the arm number 3 gives the maximum return and hence one idea is to keep playing this arm in order to obtain the maximum reward (pure exploitation).

Just based on the knowledge from the given sample, 5 might look like a bad arm to play, but we need to keep in mind that we have played this arm only once and maybe we should play it a few more times (exploration) to be more confident. Only then should we decide which arm to play (exploitation).

Exploration: gather information

Exploitation: optimal decision using current information

In our k-armed bandit problem, each of the *k* actions has an expected or mean reward given that
that action is selected; let us call this the *value* of that action. We denote the action selected on time
step t as A_{t}, and the corresponding reward as R_{t}. The value then of an arbitrary action a, denoted
q_{∗}(a), is the expected reward given that a is selected:

![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20q_%7B*%7D%28a%29%5Cdoteq%20%5Cmathbb%7BE%7D%5BR_%7Bt%7D%20%7C%20A_%7Bt%7D%20%3D%20a%5D)

If you knew the value of each action, then it would be trivial to solve the *k*-armed bandit problem: you
would always select the action with highest value. We assume that you do not know the action values
with certainty, although you may have estimates. We denote the estimated value of action *a* at time
step *t* as Q_{t}(a). We would like Q_{t}(a) to be close to q_{∗}(a).

If you maintain estimates of the action values, then at any time step there is at least one action whose
estimated value is greatest. We call these the *greedy* actions. When you select one of these actions,
we say that you are exploiting your current knowledge of the values of the actions. If instead you
select one of the *nongreedy* actions, then we say you are exploring, because this enables you to improve
your estimate of the *nongreedy* action’s value. Exploitation is the right thing to do to maximize the
expected reward on the one step, but exploration may produce the greater total reward in the long run.

### Action-Value Function
The expected payoff or expected reward can also be called an action-value function. It is represented by Q(a) and defines the average reward for each action at *a* time *t*.

![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20Q_%7Bt%7D%28a%29%20%3D%20%5Cmathbb%7BE%7D%5BR%7CA%5D)

#### References:
[Reinforcement Learning Guide: Solving the Multi-Armed Bandit Problem from Scratch in Python](https://www.analyticsvidhya.com/blog/2018/09/reinforcement-multi-armed-bandit-scratch-python/)

[Sutton & Barto Book](http://incompleteideas.net/book/bookdraft2017nov5.pdf)
