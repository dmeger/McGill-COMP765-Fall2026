# Probabilistic Estimation for Sensing and Dynamics

Robots moving in the real world (and most other physical systems of interest) move with underlying uncertainty. This can come from imprecision in our models, or the fact that the physical world injects *alleatoric* uncertainty at a given level of representation (e.g., the robots roll a dice to decide who wins the most robux).

Rather than being able to predict a single state per time $x_t$, this means our interest is actually in computation of the distribution over states $p(x_t)$. This typically needs to be estimated from a set of inputs that includes the past controls $[u_0,...,u_t-1]$, sensory observations $[z_1,...,z_t]$, an initial guess $p(x_0)$, motion models $p(x_t|x_{t-1},u_{t-1})$ and a measurement model $p(z_t|x_t)$.

For this one section of the course, there is an excellent comprehensive textbook to follow. It's name is Probabistic Robotics by Thrun, Burgard and Fox (PR). I haven't provided this for you or made it a required text, which indicates that I believe you can obtain it electronically for this one week of the course somehow. Please use your initiative, and I will demonstrate my recommended method in lecture.

These notes will specify a list of topics that we want to cover out of the PR text:
- Section 1.3, motivation and first instance of the 3 doors example
- Within Chapter 2, much of the material can be useful for those new to robotics, but Section 2.4.3 is the key math that we will do in lecture and is therefore mainly testable.
- Within Chapter 3, we will follow the basics up to the intuition of the derivation of Kalman filters in 3.2.4 and the introduction of EKFs in 3.2.1 and 3.2.2. We will not cover the EKF derivation or the rest of the chapter from here.
- Within Chapter 4, we will lightly cover both the Histogram and Particle filters.

## Overall Key Learning Outcomes:

### Knowledge of the Input Models

Why are sensing and motion models inherently probabilistic? What are sources of noise in a few of the key elements (cameras, lidar ranging, wheel odometry, electric motors with encoders)

What does the Markov assumption mean physically, in terms of probability math? Revise (hopefully) independence and conditional independence. 

Parameters and key choices in using a Gaussian to model input distributions.

### Key Derivations: Algebra Elements and Intuitions

The Bayes Filter derivation: How is the initial setup related to our overall problem goal? Which assumptions are used to simplify? What laws of probability allow us to manipulation the expressions?

Kalman filter derivation: Understand the setup with Gaussian probability terms interacting through the filtering equations (these elements are re-used frequently in probabilistic world models). Familiarity with the analysis tools used: manipulation of Gaussian exponents, factor analysis and simplifications, derivative trick for finding the mean and variance of a quadratic exponent. Understand how linearity was used in the vanilla KF and how the EKF incorporates linearization.

Particle filter derivation: Understand the point-based representation of the distribution. The importance sampling procedure and its mapping on to the PF algorithm.

## Exercizes

(E3.1) (a) Write out the linear models for a robot made up of a "point-and-shoot" motion on the (x,y) plane. Its state is a position (x,y) and its control is a velocity vector (${\Delta}x,{\Delta}y$). It senses only vertically, as the distance to the x-axis ($z=y+noise$). Add the needed noise variance terms.

(b) Code the KF for this robot, invent some controls and measurements and show the estimates on a simple plot.

(c) Repeat with the Histogram filter, tiling square cells at an appropriate spacing to capture the distribution approximately. Be prepared for some headaches with the book-keeping of motion between cells (so perhaps do not waste a lot of time implementing this or get AI help!), but thinking it through is still useful.

(d) Repeat with the Particle filter. Try several different values for the number of particles. Enable and disable resampling to observe its effects. Play with the model parameters (noise levels, input magnitudes etc) to observe at least one case where the PF loses track and one where it tracks well through the full trajectory. What intuitive interactions are happening between the particles and the ground truth system that distinguishes these cases?

(E3.2) Suppose a robot has two sensors that follow models $z^1 = h^1(x)$ and $z^2 = h^2(x)$ and both take a reading at every time step. Does this setup still allow us to derive the Bayes Filter? What would the expressions look like?

(E3.3) We have claimed that $p(z|x)\overline{bel}(x)$ is Gaussian if both terms are, and shown the results. Verify this by sampling many values of $x$ and plotting the resulting distribution or fitting a Gaussian to the resulting samples. This can also be done for the motion integration step that computes $\overline{bel}(x)$.

(E3.4) Verify the Importance Sampling lemma by coding some concrete $f(x)$ and $g(x)$ distribution functions, sampling from $g(x)$ and computing $f(x)$ with IS correction. When is $f(x)$ restored accurately? How does the magnitude of difference between the functions impact your results? How many samples are needed for an accurate estimate?