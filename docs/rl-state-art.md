---
layout: default
---
# Reinforcement Learning
[Sutton & Barto Book](http://incompleteideas.net/book/bookdraft2017nov5.pdf)
1. **Tabular Solutions Methods**: the methods can often find exact solutions, that is, they can often find exactly the optimal value function and the optimal policy.
 * **Multi-armed Bandits**: special case with only a single state
 * **Finite Markov Decision Processes**: general problem formulation for Reinforcement Learning problems.
 * **Dynamic Programming**: method to solve FMDP. Needs a model of environment.
 * **Monte Carlo Methods**: method to solve FMDP. Model not required, conceptually simple but are not well suited for step-by-step incremental computation.
 * **Temporal-Difference Learning**: method to solve FMDP. Model not required and are fully incremental, but are more complex to analyze.
2. **Approximate Solution Methods**: in such cases we cannot expect to find an optimal policy or the optimal value function even in the limit of infinite time and data; our goal instead is to find a good approximate solution using limited computational resources.
  * **On-policy Prediction with Approximation**
  * **On-policy Control with Approximation**
  * **On-policy Methods with Approximation**
  * **Eligibility Traces**
  * **Policy Gradient Methods**

[2017- A Brief Survey of Deep Reinforcement Learning](https://arxiv.org/pdf/1708.05866.pdf)

[\\]: https://spinningup.openai.com/en/latest/spinningup/keypapers.html
[\\]: https://medium.com/@yuxili/resources-for-deep-reinforcement-learning-a5fdf2dc730f
Classical methods:
* **Model-free methods**:
  * **value-based methods**: bases upon temporal difference learning, learn value function.
    * TD
    * [Q-learning](http://www.cs.rhul.ac.uk/~chrisw/new_thesis.pdf)
    * SARSA
  * **policy-based**: directly learn optimal policy.
    * **policy search**
    * **policy gradient**:
      * REINFORCE
    * **actor-critic methods**
* **Model-based methods**:    

Deep Learning methods:
* **Model-free methods**:
  * **value-based methods**: Deep networks to represent value/Q functions. Estimate the value function, policy is implicit.
    * DQN
      * [Playing Atari with Deep Reinforcement Learning](https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf)
      * [Human-level control through deep reinforcement learning](https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf)
      * [Rainbow: Combining Improvements in Deep Reinforcement Learning](https://arxiv.org/pdf/1710.02298.pdf)
      * [Priority Experience Replay](https://arxiv.org/pdf/1511.05952.pdf)
      * [Dueling Network Architecture for Deep Reinforcement Learning](https://arxiv.org/pdf/1511.06581.pdf)
    * Double-Q
      * [Deep Reinforcement Learning with Double Q-Learning](https://www.aaai.org/ocs/index.php/AAAI/AAAI16/paper/viewFile/12389/11847)
    * Categorical DQN
      * [A Distributional Perspective on Reinforcement Learning](https://arxiv.org/pdf/1707.06887.pdf)
    * Continuous DQN (NAF (Normalised Advantage Function) or CDQN):
      * [Continuos Deep Q-Learning with Model-based Acceleration](https://arxiv.org/pdf/1603.00748.pdf)
    * [Deep SARSA](https://ieeexplore.ieee.org/document/7849837)
    * NEC ([Neural Episodic Control](https://arxiv.org/pdf/1703.01988.pdf))
  * **policy-based methods**: estimate the policy, no value function. Neuronal networks policies, more complex policy search.
    * **policy search** ([End-to-End Training of Deep Visuomotor Policies](https://arxiv.org/pdf/1504.00702.pdf)):
      * GPS ([Guided Policy Search](https://graphics.stanford.edu/projects/gpspaper/gps_full.pdf))
    * **policy gradients**([Policy Gradient Methods for RL with function approximation](https://papers.nips.cc/paper/1713-policy-gradient-methods-for-reinforcement-learning-with-function-approximation.pdf)):
      * PPO ([Proximal Policy Optimization](https://arxiv.org/pdf/1707.02286.pdf))
      * TRPO ([Trust Region Policy Optimization](https://arxiv.org/pdf/1502.05477.pdf))
      * GAE ([Generasiled Advantage Estimation](https://arxiv.org/pdf/1506.02438.pdf))
      * SVG ([Stochastic Value Gradients](https://arxiv.org/pdf/1510.09142.pdf))  
    * **Actor-critic methods** ([Visual Navigation in Indoor Scenes](https://arxiv.org/pdf/1512.01693.pdf)): estimate the policy and the value function.
      * DPG ([Deterministic Policy Gradients](https://arxiv.org/pdf/1506.02438.pdf))
      * IPG ([Interpolated Policy Gradients](https://arxiv.org/pdf/1706.00387.pdf))
      * [A4C](https://openreview.net/pdf?id=rk8YHK1wG), [A3C](https://arxiv.org/pdf/1602.01783.pdf), A2C
      * [DA2C](https://arxiv.org/pdf/1806.06914.pdf)
      * [Soft actor-critic](https://sites.google.com/view/sac-and-applications)
      * [Q-Prop](https://arxiv.org/pdf/1611.02247.pdf)
      * [ACKTR](https://arxiv.org/pdf/1708.05144.pdf)
* **Model-based methods**: Neural networks or Gaussian processes for representing dynamics, advanced optimal control for planning.
  * [One-Shot Learning of Manipulation Skills with Online Dynamics Adaptation and Neural Network Priors](https://arxiv.org/pdf/1509.06841.pdf)

Current research & challenges:
* **Model-based RL**: the key idea behind model-based RL is to learn a transition model that allows for simulation of the environment without interacting with the environment directly.
  * [Deep Spatial Autoencoders for Visuomotor Learning](https://arxiv.org/pdf/1509.06113.pdf)
* **Exploration vs. Exploitation**: one of the greatest difficulties in RL is the fundamental dilemma of exploration versus exploitation: When should the agent try out (perceived) non-optimal actions in order to explore the environment (and potentially improve the model), and when should it exploit the optimal action in order to make useful progress?
* **Hierarchical RL**: In the same way that deep learning relies on hierarchies of features, HRL relies on hierarchies of policies.
* **Imitation Learning and Inverse RL**:
  * [ALVINN](https://papers.nips.cc/paper/95-alvinn-an-autonomous-land-vehicle-in-a-neural-network.pdf)
  * [A Survey of Inverse Reinforcement Learning](https://arxiv.org/pdf/1806.06877.pdf)
* **Memory and attention**:
  * [Deep Recurrent Q-network DQRN](https://arxiv.org/pdf/1507.06527.pdf)
  * [Deep Attention Recurrent Q-network](https://arxiv.org/pdf/1512.01693.pdf)
* **Transfer Learning**:  
* **Benchmarks**: one of the challenges in any field in machine learning is developing a standardised way to evaluate new techniques.

[2019 - Deep reinforcement learning with smooth policy update: Application to robotic cloth manipulation](https://ac.els-cdn.com/S0921889018303245/1-s2.0-S0921889018303245-main.pdf?_tid=f9dfabff-69ba-48c5-860a-b7cb57a4da0c&acdnat=1550232056_12e0881bdb0b83e394a16bca20a7f7bb)

[2018 - Dexterous Manipulation with Reinforcement Learning: Efficient, General, and Low-Cost](https://bair.berkeley.edu/blog/2018/08/31/dexterous-manip/)

[2018 - Learning to Walk via Deep Reinforcement Learning](https://arxiv.org/pdf/1812.11103.pdf)

[2018 - An Introduction to Deep Reinforcement Learning](https://arxiv.org/pdf/1811.12560.pdf)

[2018 - Deep Reinforcement Learning for robotic manipulation-the state of the art](https://arxiv.org/pdf/1701.08878.pdf)

[2018 - Qt-Opt: Scalable Deep Reinforcement Learning for Vision-Based Robotic Manipulation](https://arxiv.org/pdf/1611.02247.pdf)

[2017 - Deep Reinforcement Learning for Robotic Manipulation with Asynchronous Off-Policy Updates](https://ai.google/research/pubs/pub45926)

## Definitions
### Multi-armed Bandits
[//]: https://www.analyticsvidhya.com/blog/2018/09/reinforcement-multi-armed-bandit-scratch-python/

A one-armed bandit is a simple slot machine wherein you insert a coin into the machine, pull a lever, and get an immediate reward. A multi-armed bandit is a complicated slot machine wherein instead of 1, there are several levers which a gambler can pull, with each lever giving a different return.  The probability distribution for the reward corresponding to each lever is different and is unknown to the gambler.

![Multi-armed Bandits](multi_armed_bandit_problem.png "Multi-arned Bandits")

The task is to identify which lever to pull in order to get maximum reward after a given a set of trials. This problem statement is like a single Markov decision process. Each arm chosen is equivalent to an action, which then leads to an immediate reward.

[Reinforcement Learning Guide: Solving the Multi-Armed Bandit Problem from Scratch in Python](https://www.analyticsvidhya.com/blog/2018/09/reinforcement-multi-armed-bandit-scratch-python/)

[Policy Search: Methods and Applications](https://icml.cc/2015/tutorials/PolicySearch.pdf)

## Courses
[David Silver - Pre Alpha-Go](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html)

[John Schulman videos](https://www.youtube.com/watch?v=oPGVsoBonLM)

[CoRL 2018](https://www.video.ethz.ch/events/2018/corl/c7111aff-c968-43cd-ad0f-b42ddcccbf62.html)

[Deep RL Bootcamp](https://sites.google.com/view/deep-rl-bootcamp/lectures)

[Deep RL - CV294-112 - Berkley](http://rail.eecs.berkeley.edu/deeprlcourse/)

[ICAPS - Summer School - 2018](http://icaps18.icaps-conference.org/summerschool/index.html#program)

[Practical_RL - YSDA](https://github.com/yandexdataschool/Practical_RL)

[Reinforcement Learning Explained](https://www.edx.org/es/course/reinforcement-learning-explained-3)

[Deep Reinforcement Learning Nanodegree](https://eu.udacity.com/course/deep-reinforcement-learning-nanodegree--nd893)

[OpenAI Spinning Up - rl introduction](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html)

## Blogs & Resources

[Deep Reinforcement Learning: Pong from Pixels](http://karpathy.github.io/2016/05/31/rl/)

[Demystifying Deep Reinforcement Learning](https://www.intel.ai/demystifying-deep-reinforcement-learning/#gs.0zdSLwZz)

[Simple Reinforcement Learning with Tensorflow](https://medium.com/emergent-future/simple-reinforcement-learning-with-tensorflow-part-0-q-learning-with-tables-and-neural-networks-d195264329d0)

[Let's make a DQN: Theory](https://jaromiru.com/2016/09/27/lets-make-a-dqn-theory/)

[Reinforcement Learning: Q-Learning and exploration](https://studywolf.wordpress.com/2012/11/25/reinforcement-learning-q-learning-and-exploration/)

[Awesome-RL Github repo](https://github.com/aikorea/awesome-rl)

[A Beginner's Guide to Deep Reinforcement Learning](https://skymind.ai/wiki/deep-reinforcement-learning)

[Model-based reinforcement learning](https://medium.com/@jonathan_hui/rl-model-based-reinforcement-learning-3c2b6f0aa323)

[Deep Reinforcement Learning: Playing CartPole through Asynchronous Advantage Actor Critic (A3C)](https://medium.com/tensorflow/deep-reinforcement-learning-playing-cartpole-through-asynchronous-advantage-actor-critic-a3c-7eab2eea5296)

[Actor-Critic Methods: A3C and A2C](https://danieltakeshi.github.io/2018/06/28/a2c-a3c/)

[Soft Actor Critic—Deep Reinforcement Learning with Real-World Robots](https://bair.berkeley.edu/blog/2018/12/14/sac/)

[Deep Reinforcement Learning for Robotics](http://www.fujitsu.com/us/Images/Panel3_Pieter_Abbeel.pdf)

[Google X’s Deep Reinforcement Learning in Robotics using Vision](https://hackernoon.com/google-xs-deep-reinforcement-learning-in-robotics-using-vision-7a78e87ab171)

[Controlling a 2D Robotic Arm with Deep Reinforcement Learning](https://blog.floydhub.com/robotic-arm-control-deep-reinforcement-learning/)

[Reinforcement Q-Learning from Scratch in Python with OpenAI Gym](https://www.learndatasci.com/tutorials/reinforcement-q-learning-scratch-python-openai-gym/)

[Reinforcement Learning: Introduction to Monte Carlo Learning using the OpenAI Gym Toolkit](https://www.analyticsvidhya.com/blog/2018/11/reinforcement-learning-introduction-monte-carlo-learning-openai-gym/)

## Reinforcement Learning environments
[OpenAI Gym](https://gym.openai.com/)

[OpenAI Gym Gazebo](https://github.com/erlerobot/gym-gazebo)

[OpenAI Gym Spinning Up](https://spinningup.openai.com/en/latest/index.html)

[RLenv.directory](https://rlenv.directory/)

[Coach](https://github.com/NervanaSystems/coach)

[Deep Mind Lab](https://github.com/deepmind/lab)

[DeepMind Control Suite](https://github.com/deepmind/dm_control)

[Unity ML Agents](https://github.com/Unity-Technologies/ml-agents)

[Malmo](https://github.com/Microsoft/malmo)

[bandits](https://github.com/bgalbraith/bandits)
