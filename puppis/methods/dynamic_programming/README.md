#Markov Decision Process
MDP is represented by:
* A set of states (S) the agent can actually be in.
* A set of actions (A) that can be performed by an agent, for moving from one state to another.
* A transition probability (P), which is the probability of moving from one state S to another S' state by performing some action a.
* A reward probability (R) which is the probability of a reward acquired by the agent for moving from one state S to another S' by performing some action a.
* A discount factor (gamma), which controls the importance of inmediate and future rewards. 

##Policy function
This function maps states to actions, it is denoted by pi. A policy function says what action to perform in each state. The optimal policy specifies the correct action to perform in each state, which maximize the reward.

##State value function
Or value function, it specified how good it is for an agent to be in a particular state with a policy pi. A value function is denoted by V(S). The state value function depends on the policy and it varies depending on the policy we choose.

##State-action value function (Q function)
It specified how good is for an agent to perform a particular action in a state with a policy pi. The Q function is denoted by Q(S). The Q function specifies the expected return starting from state S with the action a according to policy pi. The difference between value function and Q function is that the value function specifies the goodness of a state, while a Q function specifies the goodness of an action in a state.

##The Bellman equation
When we say solve the MDP, it actually means finding the optimal policies and value functions. Resolving the Bellman equation we are finding the optimal policies or values functions.

#Dynamic Programming
Is a technique for solving complex problems. Instead of solving complex problems one at a time, we break the problem into simple sub-problems, then for each sub-problem, we compute and store the solution. If the same problem occurs, we will not recompute, instead, we use the already computed solution.

We solve a Bellman equation using two algorithms:
* Value Iteration
* Policy Iteration