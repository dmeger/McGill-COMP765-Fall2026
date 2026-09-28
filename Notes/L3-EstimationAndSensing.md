# Probabilistic Estimation for Sensing and Dynamics

Robots moving in the real world (and most other physical systems of interest) move with underlying uncertainty. This can come from imprecision in our models, or the fact that the physical world injects *alleatoric* uncertainty at a given level of representation (e.g., the robots roll a dice to decide who wins the most robux).

Rather than being able to predict a single state per time $x_t$, this means our interest is actually in computation of the distribution over states $p(x_t)$. This typically needs to be estimated from a set of inputs that includes the past controls, sensory observations, an initial guess, a motion model and a measurement model.

For this one section of the course, there is an excellent comprehensive textbook to follow. It's name is Probabistic Robotics by Thrun, Burgard and Fox (PR). I haven't provided this for you or made it a required text, which indicates that I believe you can obtain it electronically for this one week of the course somehow. Please use your initiative, and I will demonstrate my recommended method in lecture.

These notes will specify a list of topics that we want to cover out of the PR text:
- Section 1.3, motivation and first instance of the 3 doors example
- Within Chapter 2, much of the material can be useful for those new to robotics, but Section 2.4.3 is the key math that we will do in lecture and is therefore mainly testable.
- Within Chapter 3, we will follow the basics up to the intuition of the derivation of Kalman filters in 3.2.4 and the introduction of EKFs in 3.2.1 and 3.2.2. We will not cover the EKF derivation or the rest of the chapter from here.
- Within Chapter 4:
   - Discrete (Histogram) filter material includes 4.1.1 and 4.1.2 (but not the rest of 4.1 about decomposing states)
   - Particle filter material includes 4.2.1, 4.2.2 and 4.2.3. Section 4.2.4 is really useful for you to understand the above in more detail, but is not strictly testable in the course. 
   - 4.3 is a helpful summary, but just FYI/context.


## Overall Key Learning Outcomes:

### Knowledge of the Input Models

Why are sensing and motion models inherently probabilistic? What are sources of noise in a few of the key elements (cameras, lidar ranging, wheel odometry, electric motors with encoders)

What does the Markov assumption mean physically, in terms of probability math? Revise (hopefully) independence and conditional independence. 

Parameters and key choices in using a Gaussian to model input distributions.

### Key Derivations: Algebra Elements and Intuitions

The Bayes Filter derivation: How is the initial setup related to our overall problem goal? Which assumptions are used to simplify? What laws of probability allow us to manipulation the expressions?

Kalman filter derivation: Understand the setup with Gaussian probability terms interacting through the filtering equations (these elements are re-used frequently in probabilistic world models). Familiarity with the analysis tools used: manipulation of Gaussian exponents, factor analysis and simplifications, derivative trick for finding the mean and variance of a quadratic exponent. Understand how linearity was used in the vanilla KF and how the EKF incorporates linearization.

Particle filter derivation: Understand the point-based representation of the distribution. The importance sampling procedure and its mapping on to the PF algorithm.

## Bonus Research Topic: Decision Making Under Uncertainty

The tools above give us a selection of useful ways to estimate the belief over a robot's state. We picked the form of belief $bel(x_t)$ to be a distribution over the current state at time $t$ specifically so it can be informative for immediate decision making. It's very practically important to turn this belief into a decision, but the general problem of making optimal decisions in light of model errors, learning and varying objectives is an open research problems. We will not cover this area fully, but here are a few of the key considerations and some of the existing answers:

### Extracting a single state

The most straightforward way to make decisions after computing $bel(x_t)$ is to locate one representative state, ideally one that is likely (the most likely?) to occur. This single state summarizes the distribution and allows for the use of control solutions that require a single state as input (e.g., controllers of the forms $u={\pi}(x)$, $a=-Kx$, etc.) For Gaussian beliefs we have the best chances. The mean ${\mu}_t$ is the most likely single state due to the unimodal nature of Gaussians. All else being equal, this is a good choice for the Kalman filtering case.

Selecting a single state from the Particle or Histogram filters is more complicated. After normalization, we have washed away the weights of particles and represent density by numerical repetition, so even the weight fields may not be informative. An interesting idea is to sample, $x_{sample} \sim bel(x_t)$, such that the state we use for control is drawn fairly from the belief distribution. The expectation of our sampled variable will match the expectation of $bel(x_t)$. Will the sampled point be a highly likely state or not? If we have resampled correctly, we will be selecting the particle states in exactly proportion to their likelihood within the belief. Essentially, the less likely the particle, the less likely we are to select it, in corresponding proportion. So, this is an OK way to proceed, if we are willing to accept some extra variance from sampling. We are usually not willing, and so we might try an aggregation technique.

Option 1: Compute the expectation $x_{exp} = E_{bel(x_t)}[x_t]$. For an underlying Gaussian (or unimodal) distribution, this gives a good answer, equivalent to the case of Kalman filtering. What about more complex distributions? Does $bel(x_{exp})$ have to be high? Sadly the answer is no for any distribution that happens to have a low value at its expected value. A simple mixture of two Gaussian modes with equal weight is an example. The point right in between the two modes can have very low likelihood.

Option 2: Compute the most likely state $x_{maxbel} = argmax_{x_t} bel(x_t)$. This requires more computation in general, as for each state candidate we must aggregate over all (or some local approximation) particles and assess their contribution. This procedure is known as density estimation, and we'll have to carry it out for a range of options, taking the max over our candidates. With time and compute to spare, this is a reasonable option that is sometimes used in practice.

### Control for the whole distribution

Rather than sticking with control methods that assumed a single deterministic trajectory, one can modify the control objective to make decisions based on the full computed belief distribution. The textbook describes an aspirational, but very computationally expensive solution known as the Partially Observable Markov Decision Process (POMDP), which is primarily a good thought experiment except in very special cases where we're willing to spend extensive compute even on a tiny problem. Another special case is the combination of LQR control with a stochastic system that fits the Kalman filtering assumptions. For this case, it can be shown that the expected sum of quadratic costs is minimized by applying LQR to the expected states (means) determined by running a Kalman filter. The proof extends the use of linearity common to both LQR and KF reasoning. Geometrically, placing the mode of our Gaussian belief at the minimum of our quadratic cost leads to the minimum expected value - any offset will fairly clearly lead to higher expected cost.

In some cases, we want our decision making to be more sensitive to the uncertainty computed in the filter. This can be due to having a desired robot behavior in mind, and it is often an aspect of learning (improved stability, encourage data collection, etc.) In these cases, one can experiment with costs beyond the simple quadratic and perform some related math to analyse the interaction of each cost with the Gaussian belief, exploring the outcomes on exploration and exploitation. A good example is the PILCO method we will look at shortly [PILCO](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6654139).

Two important topics in this area that gets beyond the scope of these notes are: (1) risk-aware control and (2) active learning for model learning. I hope to revisit each either in later notes or during our paper readings.

## Exercises

(E3.1) (a) Write out the linear models for a robot made up of a "point-and-shoot" motion on the (x,y) plane. Its state is a position (x,y) and its control is a velocity vector (${\Delta}x,{\Delta}y$). It senses only vertically, as the distance to the x-axis ($z=y+noise$). Add the needed noise variance terms.

(b) Code the KF for this robot, invent some controls and measurements and show the estimates on a simple plot.

(c) Repeat with the Histogram filter, tiling square cells at an appropriate spacing to capture the distribution approximately. Be prepared for some headaches with the book-keeping of motion between cells (so perhaps do not waste a lot of time implementing this or get AI help!), but thinking it through is still useful.

(d) Repeat with the Particle filter. Try several different values for the number of particles. Enable and disable resampling to observe its effects. Play with the model parameters (noise levels, input magnitudes etc) to observe at least one case where the PF loses track and one where it tracks well through the full trajectory. What intuitive interactions are happening between the particles and the ground truth system that distinguishes these cases?

(E3.2) Suppose a robot has two sensors that follow models $z^1 = h^1(x)$ and $z^2 = h^2(x)$ and both take a reading at every time step. Does this setup still allow us to derive the Bayes Filter? What would the expressions look like?

(E3.3) We have claimed that:

$$\begin{aligned}
p(z|x)\overline{bel}(x)
\end{aligned}$$ 

is Gaussian if both terms are, and shown the results. Verify this by sampling many values of $x$ and plotting the resulting distribution or fitting a Gaussian to the resulting samples. This can also be done for the motion integration step that computes 

$$\begin{aligned}
\overline{bel}(x)
\end{aligned}$$

(E3.4) Verify the Importance Sampling lemma by coding some concrete $f(x)$ and $g(x)$ distribution functions, sampling from $g(x)$ and computing $f(x)$ with IS correction. When is $f(x)$ restored accurately? How does the magnitude of difference between the functions impact your results? How many samples are needed for an accurate estimate?