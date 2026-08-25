# Reinforcement Learning

While the previous section described very useful tools to understand and control robots, we can note that there was no "learning" happening. We didn't need to collect any data and we did require full model knowledge: that is, the MDP state space, action space, transition function and reward function were needed as inputs to Iterative Policy Evaluation and Value Iteration.

Learning in this type of system means trial-and-error: making behaviors, observing their outcomes and using the generated data to solve for components such as the Value function or policy. The data that a Reinforcement Learner would receive are tuples (s,a,r,s'), where the $a=\pi(s)$ for some behavior policy that was used to collect the data. This may be the same, or may be different from the learner's current best guess at the optimal behavior currently, for reasons of computation or exploration. This distinction makes learners:
- On-policy: when the data they learn from is drawn such that $a=\pi(s)$ with the current $\pi$ under consideration, or
- Off-policy: when the actions in the data can be from a different $\pi$.

## Q-Learning Off-Policy RL for Discrete State/Action

When we lack knowledge of the transition and reward model, the knowledge of $V(s)$ alone is insufficient to select optimal actions. We are no longer able to assign the proper weighting of $V(s_{t+1})$ over possible next states, which we used the known transition function for in Optimal Control. So, what needs to change? We have to capture the value of being in a state and taking an action (this will allow a max over actions to pick optimal behavior). Our new construct is called the Action-Value function, and written as:

$$\begin{align}
Q^{\pi}(s,a) &=& r(s,a) + \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s,a)} \sum_{k=1:\infty}{\gamma}^{k} r(s_{t+k},a_{t+k})\\
&=& r(s,a) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s,a)} \sum_{k=1:\infty}{\gamma}^{k-1} r(s_{t+k},a_{t+k})\\
&=& r(s,a) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s,a)}  Q^{\pi}(s_{t+1},\pi(s_{t+1})).
\end{align}$$

This final line is Equation (3), the Action-Value Bellman Equation. An optimal variant is easy to write down. Equation (4) below are the Action-Value Bellman Optimality Equations:

$$\begin{align}
Q^{*}(s,a) &=&r(s,a) + \gamma  max_{a'} \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s,a)}  Q^{*}(s_{t+1},a').
\end{align}$$

Equation (4) is the basis of our next method, Q-Learning. We once again intitialize a $Q(s,a)$ vector at random (zeros?) and then proceed to update, this time from the data we've collected from the system. Every time we obtain a tuple $(s,a,r,s')$, we run the update to the $Q(s,a)$ suggested in Equation (4): $Q(s,a) = r(s,a) + \gamma  max_{a'}Q(s',a')$. Note that we miss the expectation from this line, as that's not available to us without model knowledge. But, the data we used to do the update included $s'$, which is a valid sample from the probability over which we wanted the expectation, $p(s_{t+1}\|s,a)$. Therefore, doing this update repeatedly on observed data ends up being a valid learning approximation and converges to $Q^{*}(s,a)$ when we've seen enough data gathered by the best policy we have at the moment, plus some small exploration.

(NOTE, this document is being actively typed and added to. Refresh in a few days to find more content.)