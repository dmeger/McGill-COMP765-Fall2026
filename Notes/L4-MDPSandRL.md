# Markov Decision Processes 

Markov Decision Processes (MDP) and Reinforcement Learning are parallel fields to Optimal Control, which occur more primarily in Computer Science and often focus on discrete state and action spaces. 

The same models and objectives hold. The dynamics are $p(s'\|s,a)$ (if deterministic, $s'=f(s,a)$). We use a reward instead of a cost, $r(s,a) = -c(s,a)$ The control policy is $a=\pi(s)$ and the overall objective is:

$$\begin{aligned}
J(\pi)=\mathbb{E}_{s_0 \sim p(s_0)}\sum_{t=0:\infty}{\gamma}^t r(s_t,a_t).
\end{aligned}$$

## Policy Evaluation

Q: What is the objective value of a given policy, $\pi$, that is, the scalar output of $J(\pi)$?

A1: It is an empirically evaluatable quantity. We could just run the policy many times and take a simple average of the discounted sums of returns. This is correct mathematically and statistically, but impossible computationally. The sum within J runs to infinity, so we cannot actually run this full procedure to completion. Even in worlds where we know we will eventually reach some terminating state, it might take unreasonably long. What are other options?

A2: Divide and conquor the $J$ expression by noting there is a relationship between the subsequent terms in the sum that is fully determined by the MDP model. To see it, we define a sum of discounted future returns conditioned on the process starting in a given state, $s$ and acting based on policy $\pi$ from then onwards.

$$\begin{aligned}
V^{\pi}(s) = r(s_t,\pi(s_t)) + \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,\pi(s_t))}\sum_{k=1:\infty}{\gamma}^k r(s_{t+k},a_{t+k}).
\end{aligned}$$

Note that the ${\gamma}^{k}$ term is a multiple of all terms in the sum, with $k>1$ in all cases. We can factor one $\gamma$, leveraging linearity of sums and expectation to reach:

$$\begin{aligned}
V^{\pi}(s) = r(s_t,\pi(s_t)) + \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,\pi(s_t))}\gamma \sum_{k=1:\infty}{\gamma}^{k-1} r(s_{t+k},a_{t+k}).
\end{aligned}$$

This reduces the discount order of the initial term in the sum to 0, which we can identify as another copy of the Value function. Define Equation (1) as:

$$\begin{aligned}
V^{\pi}(s) = r(s_t,\pi(s_t)) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,\pi(s_t))} V^{\pi}(s_{t+1}). 
\end{aligned}$$

The above set of equations, one for each state, are called the Bellman Equations. They are the key tool across all of Reinforcement Learning. Every correct solution to the Policy Evaluation problem must satisfy the Bellman Equations. Therefore, they define a system of linear equations. They could be solved directly by inverting the linear operator relating the left and right-hand sides, but this is expensive for large state spaces and leaves little potential for integration with control, which is our final goal.

Instead, iterative Policy Evaluation means starting with an initial guess for $V(s)$ across all states. Then, we loop over the states, evaluating Equation (1) each time. This procedure is guaranteed to converge to the accurate values for all states by contraction reasoning (the proof lives in a full RL course or the RL text).

## Value Iteration

While we now have a way to compute $V^{\pi}(s)$ for every policy, $\pi$, we want to go further and find the optimal behavior policy, ${\pi}^{\*}$, which is defined mathematically as $argmax_{\pi}J(\pi)$. Since Value functions capture portions of the infinite sums in $J$, we can express this optimal policy's value in Equation (2), as:

$$\begin{aligned}
V^{*}(s) = max_{a}\large[ r(s_t,a) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,a)} V^{*}(s_{t+1}) \large]. 
\end{aligned}$$

Equation (2) is known as the Bellman Optimality Equations. They are equivalent to Equation (1)'s equations when the policy, $\pi$ in $V^{\pi}$ is optimal but have the benefit of being true without knowing the policy! Therefore, they open up our ability to write algorithms that operate purely in the space of Value functions. One of the most famous is called Value Iteration.

Value Iteration is the optimality analog of iterative Policy Evaluation. We randomly initialize a $V(s)$ (perhaps zero for every state). Then, we loop over each entry in the value vector, evaluating the right hand side of Equation (2) for each. Surprisingly, this procedure is guaranteed to converge, and when it does, the $V(s)$ vector holds $V^{*}(s)$. 

Why? The argument is based on contraction logic. The maximum change that will occur for any state in a given loop shrinks by $\gamma$ compared to previous applications. Eventually, the updates must become less than any fixed constant $\epsilon$, and we have computed (within $\epsilon$-accuracy) the unique $V^{*}$.

### Optimal Policy Extraction

Note that we said Value Iteration was for control, but only computing $V^{*}$ may not seem to allow us to behave optimally at first. Happily, the definition of the optimal value, plus some model knowledge allows optimal action, with the rule for picking actions at every state (policy): 

$$\begin{aligned}
{\pi}^{*}(s)=argmax_a \large[r(s,a) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s,a)}V^{*}(s_{t+1}) \large].
\end{aligned}$$ 

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

## Continuous State-Action Methods

Each approach above expected to be able to update the value functions at a finite number of state/action pairs. This makes several things possible:
- sweeping over all state candidates for model-based approaches;
- computing the (finite) set of next states(actions/rewards) to calculate the right-hand side in update equations;
- explicit maximization over the available actions for each state.
For more naturally robotics problems with both continous states and actions, none of these is possible. We can still utilize the Bellman equations to form MDP solving and RL algorithms, but modification is needed.

### The Policy Gradient Theorem

The main element of the previous methods that must be replaced is the ability to implicity extract a policy by maximization over Q. Instead, the Policy Gradients (PG) approach uses calculus to find improvements on an explicit parameterized policy function ${\pi}_{\theta}(a \| s)$. This begins by manipulating the definition of the policy value function:

$$\begin{aligned}
\nabla v_{\pi}(s) = \nabla \large[ {\Sigma}_a {\pi}_{\theta}(a | s) {q}_{\pi}(s,a)\large]\\ 
\sim {\Sigma}_s \mu(s) {\Sigma}_a {\nabla}{\pi}_{\theta}(a | s) {q}_{\pi}(s,a)
\end{aligned}$$

These lines hold a few manipulations that push the gradient within the sum and address $\nabla q$ term that appears when applying the chain rule. The proof in Appendix 1 of the [PGT paper](https://proceedings.neurips.cc/paper_files/paper/1999/file/464d828b85b0bed98e80ade0a5c43b0f-Paper.pdf) is helpful reading. While it won't be pleasant to compute $\mu(s)$ exactly for every policy we consider, this result points to sampling as a good candidate for computation, since running the policy $\pi$ live on the system can bring us states that are sampled from $\mu$. This suggests an update like:

$$\begin{aligned}
= \mathbb{E}_{\pi} {\Sigma}_a {\nabla}{\pi}_{\theta}(a | S_t) {q}_{\pi}(S_t,a)
\end{aligned}$$

for samples $S_t$ drawn using the policy. Notably, we still have to compute a sum over all actions in this update, which is impractical for continuous action spaces. This leads us towards our actual practical methods.

### Example PGT Method 1: REINFORCE

The exact idea above, when Monte-Carlo returns are used to estimate $q$ is called REINFORCE. The updates are:

$$\begin{aligned}
{% raw %}
{\nabla} J \sim \mathbb{E}_{\pi} {\Sigma}_a {\pi}_{\theta}(a|S_t) {q}_{\pi}(S_t,a) \frac{{\nabla}{\pi}_{\theta}(a | S_t)}{{\pi}_{\theta}(a|S_t)}\\
= \mathbb{E}_{\pi} {q}_{\pi}(S_t,A_t) \frac{{\nabla}{\pi}_{\theta}(A_t | S_t)}{{\pi}_{\theta}(S_t,A_t)}\\
= \mathbb{E}_{\pi} G_t \frac{{\nabla}{\pi}_{\theta}(A_t | S_t)}{{\pi}_{\theta}(S_t,A_t)}
{% endraw %}
\end{aligned}$$

where $G_t$ is the Monte-Carlo return and the capitalized variables represent samples of states and actions. This gradient can be directly used to update the policy parameters, but it happens to have a high variance in practice due to the use of the full return.

### Example PGT Method 2: Actor Critic

An updated method using the idea of TD bootstrapping is idea of maintaining a Q function, updated in the usual online RL fasion:

$$\begin{aligned}
Q(s,a) \sim r + \gamma Q'(s',a')
\end{aligned}$$

and using this as the estimate to update the policy under PGT. The Actor Critic (AC) update is:

$$\begin{aligned}
\nabla J = Q(S_t,A_t) \nabla {\pi}(A_t|S_t)
\end{aligned}$$

This method is used frequently in low dimensional RL for continuous problems and is also the primary basis for most Deep RL methods that can be used for robotics. We will see its heavy use in Wold Model learning approaches.

## Conclusion

This concludes our exceptionally brief tour through MDP modeling and RL. It is intended as a companion to the world modeling content that we will beging in the next weeks, sufficient to help you understand the terminology and basic algorithms that will be pair with world model learning to form integrated learning and control systems.

# Exercizes

(E4.1) Consider the uniqueness of $Q^*$ vs ${\pi}^*$. For each prove that there is a distinct (single) solution, or create a counter-example where multiple solutions exist.

(E4.2) What are the computational costs of each of the algorithms described in this section? Consider the sizes of the value and policy representations used and the order of the algorithmic processing elements.

(E4.3) Code a simple environment, such as a navigational grid with dynamics that allows (noisily) moving to neighboring cells. Mark one state as the start and a reward only upon reaching a goal. Implement Value Iteration and Q-Learning for this problem.