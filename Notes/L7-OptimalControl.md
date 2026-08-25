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

Note that the ${\gamma}^{k}$ term is a multiple of all terms in the sum, with $k>1$ in all cases. We can factor one $\gamma$ to reach:

$$\begin{aligned}
V^{\pi}(s) = r(s_t,\pi(s_t)) + \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,\pi(s_t))}\gamma \sum_{k=1:\infty}{\gamma}^{k-1} r(s_{t+k},a_{t+k}).
\end{aligned}$$

This reduces the discount order of the initial term in the sum to, 0, which we can identify as another copy of the Value function. Define Equation (1) as:

$$\begin{aligned}
V^{\pi}(s) = r(s_t,\pi(s_t)) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,\pi(s_t))} V^{\pi}(s_{t+1}). 
\end{aligned}$$

The above set of equations, one for each state, are called the Bellman Equations. They are the key tool across all of Reinforcement Learning. Every correct solution to the Policy Evaluation problem must satisfy the Bellman Equations. Therefore, they define a system of linear equations. They could be solved directly by inverting the linear operator relating the left and right-hand sides, but this is expensive for large state spaces and leaves little potential for integration with control, which is our final goal.

Instead, iterative Policy Evaluation means starting with an initial guess for $V(s)$ across all states. Then, we loop over the states, evaluating Equation (1) each time. This procedure is guaranteed to converge to the accurate values for all states by contraction reasoning (proof outside the scope of 417).

## Value Iteration

While we now have a way to compute $V^{\pi}(s)$ for every policy, $\pi$, we want to go further and find the optimal behavior policy, ${\pi}^{\*}$, which is defined mathematically as $argmax_{\pi}J(\pi)$. Since Value functions capture portions of the infinite sums in $J$, we can express this optimal policy's value in Equation (2), as:

$$\begin{aligned}
V^{*}(s) = max_{a}\large[ r(s_t,a) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s_t,a)} V^{*}(s_{t+1})\large]. 
\end{aligned}$$

Equation (2) is known as the Bellman Optimality Equations. They are equivalent to Equation (1)'s equations when the policy, $\pi$ in $V^{\pi}$ is optimal but have the benefit of being true without knowing the policy! Therefore, they open up our ability to write algorithms that operate purely in the space of Value functions. One of the most famous is called Value Iteration.

Value Iteration is the optimality analog of iterative Policy Evaluation. We randomly initialize a $V(s)$ (perhaps zero for every state). Then, we loop over each entry in the value vector, evaluating the right hand side of Equation (2) for each. Surprisingly, this procedure is guaranteed to converge, and when it does, the $V(s)$ vector holds $V^{*}(s)$. 

Why? The argument is based on contraction logic. The maximum change that will occur for any state in a given loop shrinks by $\gamma$ compared to previous applications. Eventually, the updates must become less than any fixed constant $\epsilon$, and we have computed (within $\epsilon$-accuracy) the unique $V^{*}$.

### Optimal Policy Extraction

Note that we said Value Iteration was for control, but only computing $V^{*}$ may not seem to allow us to behave optimally at first. Happily, the definition of the optimal value, plus some model knowledge allows optimal action, with the rule for picking actions at every state (policy): 

$$\begin{aligned}
{\pi}^{*}(s)=argmax_a \large[r(s,a) + \gamma \mathbb{E}_{s_{t+1} \sim p(s_{t+1}|s,a)}V^{*}(s_{t+1}) \large].
\end{aligned}$$ 



