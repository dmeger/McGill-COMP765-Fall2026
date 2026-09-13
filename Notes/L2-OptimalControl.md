# Robotic Control

Distinct from planning, robot control indicates the problem of determining instantaneous control actions from instantaneously observed states $u_t = \pi(x_t)$. We may or may not deliberate about a long horizon behavior in order to come up with $\pi$, but when it comes to applying the function to run the robot, it is meant to be a "pause and deliberate" but rather an "act now, act fast" sort of outcome. However, as we'll see, computing the form, weights, code or structure of $\pi$ could still take plenty of deliberation, only it should happen in advance, or between episodes, etc.

## Example Control Problems

1) The block on ice is a unit mass object sliding without friction on the x line. Its control goal is to be at rest at $x=0$, but it is a real physical object and moves with some momentum, having acceleration coupled to the force applied, $\ddot{x} = u$, $\dot{x} = \dot{x}(0) + ut$ and $x = x(0) + \dot{x}(0)t + ut^2$ (by integration in time). 

2) The friction-less pendulum system is one which only feels forces from gravity around its single attachment point. Leverage traslates the gravitational force into a torque, $-mlg\sin(\theta)$, for mass $m$ and length, $l$. Balancing this with the 2nd form definition of torque, change in angular momentum, $\frac{dL}{dt}$, and writing out the angular momentum, $L=r \times p = ml^2\frac{d\theta}{dt}$ lets us take the derivative $\frac{dL}{dt} = ml^2\frac{d^2\theta}{dt^2}$. We can equate the two formulas for torque: $-mlg\sin(\theta) = ml^2\frac{d^2\theta}{dt^2}$ and find the overall motion equations $\frac{d^2\theta}{dt^2} + \frac{g}{l}\sin(\theta) = 0$. We might be familiar with the motion of a pendulum swinging slowly near it's downward point. When we have a "small angle", we can assume $\sin(\theta) = \theta$, which simplifies our equations to $\frac{d^2\theta}{dt^2} + \frac{g}{l}\theta = 0$. These are now equivalent to a mass oscillating on a simple spring, with a sinusoidal state trajectory in time. How about the full non-linear pendulum's time trajectories? They are not easy to write out in general, as we can see by the behavior of the phase-space diagram:

![Pendulum Phase Space](../Images/pendulum.png)

## Taking Control of the Phase Space

The fact that a physical dynamic system moves in time following the rules of its motion equations (visualized by the phase space), is kind of inconvenient when working with them, but we must embrace it and come up with ways for our robot's controllers, $u=\pi(x)$ to move the system as we desire. A first strategy can be direct analysis (by hand) of the phase space.

In example one, our block on ice can be made more interesting by limiting the controls possible to the range $[-1,1]$ and seeking a control strategy that reaches the goal at $(0,0)$ in as little time as possible (and then stay there forever). We can either accelerate to the left $u=-1$, passively drift $u=0$ or accelerate to the right $u=+1$. Each of these motions will yield a different phase space, and by mastering the three and making smart choices about where to apply each value of $u$, we can directly engineer good behavior.

The three phase plots look as follows:

![Zero Control Phase](../Images/zerou.jpg)

![Pos Control Phase](../Images/posu.jpg)

![Neg Control Phase](../Images/negu.jpg)

Notice that the $u=0$ choice does not exert any control over the system. It is simply continuing on whatever initial $\dot{x}$ path it began on.

Both $u=+1$ and $u=-1$ options exert a quadratic form (de/ac)celeration on the system. In one half-plane they slow-down and another speed-up the block. If we continuously apply the same control, the block will always eventually shoot off to infinity in the positive or negative direction.

### Merging the Controls, Bang-Bang Analysis

The key to useful actions in this system is to note that there is one quadratic in each of the $+/-1$ cases that exactly reaches the goal at the origin of $x=0$ and $\dot{x}=0$. These are highlighted in red in the diagrams. If we could ever be exactly on either red $x/\dot{x}$ trajectory, we would have a nice (optimal?) option to solve our control problem, that is simply applying the indicated control while we follow the red line, and swapping to $u=0$ exactly at the goal.

How can we take this 1-D insight embedded in the phase plane to make a solution everywhere? We need to compare and intersect the quadratics from the $u=+1$ and $u=-1$ cases. Notice that everything "below" the two red quadratics can be positively accelerated to follow a quadratic in the $u=+1$ case, and every one of those will intersect with the $u=-1$ quadratic in the top-left quadrant somewhere. Symmetrically, whenever we are above the two quadratics, there are $u=-1$ paths that negatively accelerate the block until it hits the $u=+1$ critical (red) quadratic in the bottom right quadrant.

Let's put this together: we can make a global solution up from:

- Whenever we are above the critical curve, apply control $u=-1$ until we meet the $u=+1$ curve, then switch.
- Whenever we are below the critical curve, apply control $u=+1$ until we meet the $u=-1$ curve, then switch.
- On either critical curve, apply the indicated control to follow the path towards the goal at $(0,0)$.
- Whenever we are at the goal, apply $u=0$, stop and complete the task.

The resulting phase space when acting with these rules looks like this:

![Opt Control Phase](../Images/optu.jpg)

This control method is called **Bang-Bang** because it is made up of only maximal controls and switches immediately. There is never a case where we apply $u=0.5$ or any other intermediate control.

### Thm: Bang-Bang Control is Optimal for the Minimum Time Block on Ice Problem

We can do some time integration analysis here, considering ways that one could reach from a non-zero $x$ towards zero. 
- If we are at a point such that we cannot "stop in time", then any acceleration that causes further "overshooting" is wasted effort and time. We will always have to stop and turn around eventually and out of all the $\dot{x}=0$ positions, we'd like to be at the one as close to the goal as possible. 
- If we are at a point where we need to speed up, move towards the goal and slow down (such as any point with $\dot{x}=0$ or simply away from the critical lines), we can compare doing this as fast as possible to other options like gaining a little speed and coasting, following sub-critical quadratics and more. All of these will have less than or equal velocity to the critical Bang-Bang motion of accelerating to the critical line and then decelerating perfectly to the goal. So, they waste time.

Overall, any non Bang-Bang motion can be ruled out as taking too long with the correct application of this basic argument (it either follows a path through position-space that's too long, or it follows the right positions too slowly.)

# PID and LQR Controllers

This section continues our exploration of control methods to describe two of the most widely used simple methods that are used across robotics and engineering, PID and LQR control.

## Proportional, Integral, Derivative (PID) Control

PID control is a method that applies intuitive, human designed control logic in a straightforward fashion to control a wide variety of simple systems. We build up 3 forms of control input that can accomplish different and synergystic outcomes:

- **Proportional** terms act directly on control error, $u=K_p(x_g-x)$, where $K_p$ is a user-specified gain that controls the relative magnitude of the proportional contribution. The proportional contribution aims to minimize error. It gets the system moving "towards" the goal, in the usual case that the control is some force-like quantity aligned with the state. That is, a positive force moves the system in the positive direction and vice-versa. Increasing the proportional gain means that the system will respond faster, but at some point, we expect the force might become too large and build momentum that is not canceled by drag etc, in which case we may over-shoot our target.

- **Differential** terms act on the derivative of the error $e=(x_g-x)$, $u=K_d\frac{de}{dt}$. This term is meant to fight overshoot and oscilation. We do want the error to reduce, but when the system builds up a significant momentum, we damp this out artificially with the **D-term**, such that we may smoothly hit our target. Increasing the $K_d$ gain may lead to slower response and if it is too large, we may never be able to solve the control problem, but a reasonable value is a sensible choice to make effective, smooth control motions.

- **Integral** terms fix a unique problem not addressed by P or D terms, that is steady-state error. For example, gravity is a common cause of robot systems being persistently below their control targets. If we consider a PD system under gravity, there is no component in the system that pre-computes an upward compensating force, and so only after gravity has pulled us down a bit will we see the positive error $e$ that causes us to push back. The equilibrium of this process is below the goal state. On the contrary, the integral term sums up the errors so far $u=K_i \int e$. If we have been operating a while and always had positive errors, the integral term will build, causing us to push more strongly upwards until the equilibrium reaches the goal, at which point the integral stops increasing and we may find an equilibrium with no steady-state error.

### PID Tuning

A critical part of making a PID system work is the selection of $K_p$, $K_i$ and $K_d$. The slides have a few heuristics and guidlines. There is not much to analyze carefully here in our written notes, and this is not a highly testable part of control, so we will simply guide you to listen to the video and slides for understanding.

## Optimal Control's Most Important Algorithm to Know: Linear Quadratic Regulators (LQR)

Assuming a large amount of knowledge about our robotic system and its goals, it is sensible to think of directly optimizing for the controller that will perform best on a given task over a long horizon. To start, we assume full knowledge of a simple form and parameters for all of the following:

- The system's linear dynamics: $x_t = Ax_{t-1} + Bu_{t-1}$
- The task's quadratic objective: $J = \sum_{t=0}^H{x_t^TQx_t + u_tRu_t}$

### Claim
The optimal controller for this system is a linear, time-varying matrix: $$u_t^* = -K_tx_t$$ solveable in closed-form. We will verify and demonstrate this fact by simple construction.

### The Start: $Q_{H}$, a simple quadratic 
Our task objective has a special point at the end, when the episode is about to terminate and control can no longer make a difference. We'll define $Q_H(x,u) = x_H^TQx_H=x_H^TP_Hx_H$ as the state-action value function, which is the cost to go at time $H$, making action $u$ (which is ignored), so therefore we only have a terminal Q/state cost here. We are (uselessly?) re-defining the constant matrix $Q$ as $P$ to note its role as the coefficient matrix in the final-cost quadratic equation.

### Backwards in Time: $Q_{H-1}$

Let's move one time-step backwards, to $H-1$, where we have one control left to make and must pay a cost for the $H-1$ and $H$ timesteps. We'll start to make progress by writing out this sum explicitly:

$$\begin{aligned} Q_{H-1}(x,u) &=& x_{H-1}^TQx_{H-1} + u_{H-1}^TRu_{H-1} + Q_H(x_H,u_H)\\
&=& x_{H-1}^TQx_{H-1} + u_{H-1}^TRu_{H-1} + x_{H}^TQx_{H}.
\end{aligned}$$

These three terms are simple and allow some nice analysis with calculus. First, we can expand $x_H$ using our known linear dynamics, $x_H=Ax_{H-1}+Bu_{H-1}$. This can substitute into the 3rd term above, followed by simple expansion:

$$\begin{aligned} Q_{H-1} &=& x_{H-1}^TQx_{H-1} + u_{H-1}^TRu_{H-1} + x_{H}^TQx_{H} \\
&=& x_{H-1}^TQx_{H-1} + u_{H-1}^TRu_{H-1} + (Ax_{H-1}+Bu_{H-1})^TQ(Ax_{H-1}+Bu_{H-1}) \\
&=& x_{H-1}^TQx_{H-1} + u_{H-1}^TRu_{H-1} + x_{H-1}^TA^TQAx_{H-1} + 2u_{H-1}^TB^TQAx_{H-1} + u_{H-1}^TB^TQBu_{H-1}. 
\end{aligned}$$

This mess can be made manageable by noticing we only care about finding $u^*$. The mess is at least obviously quadratic in $u$, with positive-definite coefficient matrices. It's minimum occurs where $\frac{\partial{J}}{\partial{u_{H-1}}}=0$. We continue... 

$$\begin{aligned}
\frac{\partial{J}}{\partial{u_{H-1}}} &=&  2Ru_{H-1} + 2B^TQAx_{H-1} + 2B^TQBu_{H-1}. 
\end{aligned}$$

This is zero when (grouping terms with $u_{H-1}$ on LHS):
$$\begin{aligned}
Ru_{H-1} + B^TQBu_{H-1} &=& -B^TQAx_{H-1} \\
 (R + B^TQB)u_{H-1} &=& -B^TQAx_{H-1}  \\
u_{H-1} &=& -(R + B^TQB)^{-1}B^TQAx_{H-1} \\
u_{H-1}^* &=& -K_{H-1}x_{H-1}.
\end{aligned}$$

We have accomplished the form of our claim for one very special time-step! It's time to tidy things up and get ready to attempt the solution for all $H-2$ previous time-steps.

We can plug the new form for $u$ into our expression for $Q_{H-1}$. 

$$\begin{aligned} Q_{H-1}(x,u) &=& x_{H-1}^TQx_{H-1} + u_{H-1}Ru_{H-1} + x_{H-1}^TA^TQAx_{H-1} + 2u_{H-1}^TB^TQAx_{H-1} + u_{H-1}^TB^TQBu_{H-1} \\
&=& x_{H-1}^TQx_{H-1} + x_{H-1}^TK^TRKx_{H-1} + x_{H-1}^TA^TQAx_{H-1} - 2x_{H-1}K^TB^TQAx_{H-1} + x_{H-1}K^TB^TQBKx_{H-1} \\
&=& x_{H-1}^T( Q + K^TRK + A^TQA-2K^TB^TQA + K^TB^TQBK )x_{H-1} \\
&=& x_{H-1}^TP_{H-1}x_{H-1}.
\end{aligned}$$

The final line is a simple quadradic cost in $x_{H-1}$, where we  define $P_{H-1}$ to be the appropriate coefficient matrix. Finally we can make use of the previously useless seeming statment, which we will now usefully restate: $Q_H(x,u) = x_H^TP_Hx_H$. The form is the same! 

So, consider what will happen when we write out $Q_{H-2}(x,u)$. We're going to stop spamming lists of symbols and apply our left brains here:
- $Q_{H-2}(x,u)$ can be formed of three terms, immediate state, immediate control and next state quadratic. The pattern is identical to the one we completed.
- We can apply the known linear dynamics to expand the next state, expand, take derivative and set to zero.
- Our answer will be $u_{H-2}^* = -K_{H-2}x_{H-2}$. 
- We can plug back in to the expanded form of $J_{H-2}$ and factor again to a new quadratic, $x_{H-2}^TP_{H-2}x_{H-2}$.

Generalize. There are two alternating forms that will go back to the start of the episode:

$$\begin{aligned}
J_t &=& x_t^TP_tx_t \\
u_t^* &=& -K_tx_t.
\end{aligned}$$

Our claim is supported!

## Non-linear dynamics, General-form costs and Constraints

Real robots are not linear, as we have discussed previously. The general form for dyanmics is $x_t = f(x_{t-1},u_{t})$. We can still use the technique of dynamic programming we applied above to break-down the overall objective into instantaneous and future sum of costs. The derivative will not allow closed-form solution for $u^*$ in almost any case. Taylor expansion around sensible guesses of the parameters is possible, which leads to a method known as [Iterative LQR](https://www.scitepress.org/papers/2004/11439/11439.pdf) (similar [Differential Dynamic Programming by Jacobsen and Mayne](https://www.sciencedirect.com/science/chapter/bookseries/abs/pii/B9780120127108500108). The normal problems of linearization exist; if we choose the wrong linearization point or update our parameters too far from this point, computations lose accuracy.

An important analysis tool are variational principles that define properties of optimizing solutions. For problems with sufficient structure, these tools can allow direct solution in parametric form, but for arbitrary problems, we must rely on computation.

Therefore, several components and considerations are common:
- How to consider a plausible set of points at which to evaluate our system. Ideas here can be called shooting and co-location.
- How to perform control updates including ideas like line searches, conjugate gradients, relative entropy regularization and projected gradients. 
- How to handle constraints and incorporate them into state trajectories and satisficing controls. Includes concepts such as Lagrange multipliers, dual and slack methods.

We will not dive further into the very large and active area of designing and implementing non-linear optimal control solutions in this introductory portion of the course. As we move to advanced topics, we'll see that today, Deep RL approaches have the potential to be used on the problem, especially when model knowledge is missing or unreliable. However, when we do know aspects of the model well, even a large network with lots of data can be assisted by warm-starting or other guidance from model guidance in some form. More on this to come!

## Exercizes

(Ex 2.1) Consider a rocket-powered hockey puck sliding back and forth on 1D ice. The state contains $x$ and $\dot{x}$ and control is a direct acceleration $u=\ddot{x}$. 

$$x_t = \begin{bmatrix} 1 & 1 \\ 0 & 1\end{bmatrix}x_{t-1} + \begin{bmatrix} 0 \\ 1 \end{bmatrix}u_{t}$$

Implement an LQR solution for this problem (or use a Python library). Explore the parameters Q and R and observe their effect on the outcome control solution.

(Ex 2.2) Extend the formulation to a 2D rocket-puck. You must be able to generate controls in any global direction (that is, do not include a model of puck rotation), to maintain the linear nature of the system. What trajectories can you make the puck follow and does it reach the goal at (0,0)?

(Enrichment - Not examinable) Read about how the time varying finite horizon solution we described here can be extended to an infinite horizon solution using Arithmetic Riccati Equation analysis. 
