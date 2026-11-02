# Generative Models

This lecture follows very closely to the technical discussion and derivations in several important papers:

- [PILCO](https://mlg.eng.cam.ac.uk/pub/pdf/DeiRas11.pdf)'s use of a Gaussian Process to learn a dynamics model (the policy optimization aspects of PILCO were discussed in L5).

- [Auto-Encoding Variational Bayes](https://arxiv.org/abs/1312.6114) as our introduction of latent-variable modeling and evidence lower bounds.

- [Principles of Diffusion Book](https://the-principles-of-diffusion-models.github.io/)
   - Chapter 1, for the general form of learning a probability model to match data distrubtions and overall discussion
   - Chapter 2, mainly 2.2:
      - 2.2.1: The explanation of forward and reverse diffusion and the derivation of DDPMs.
      - The intuitive analysis of the reverse denoising process in 2.2.2
      - The use of a Gaussian parameterization for reverse transitions as in 2.2.3.
   - Essentially nothing else from the book will be testable, although we had very brief discussions of Score, Flow and Drifting.
