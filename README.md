# 6DoF-SVC_model-IN1_Vestibular
6DoF-SVC (In1) model for Vestibular Motion Sickness

This is a test program for a motion sickness model that estimates the motion sickness incidence (MSI) [%] based on the input of head-centered linear acceleration (GIF) f (=g+a) and angular velocity omega.
The model implemented here is referred to as "IN1," one of the 6DoF-SVC models described in references [1] and [2]. It is based on the Subjective Vertical Conflict (SVC) theory (Bles & Bos, 1998), which can be regarded as a refined version of the well-known Sensory Conflict Theory and Neural Mismatch Theory.

The program computes MSI in two main steps:

1) Computation of the initial state vector x(0):
Assuming the human body remains in a constant posture (i.e., the same input persists over time), the state vector x is iteratively computed until it converges. Any arbitrary initial value for x can be used for this step, as convergence is guaranteed (see [2]).

2) Main simulation:
Using the converged x from step 1) as the initial state (with MSI reset to zero), the program proceeds to compute the state vector x-including MSI-sequentially in response to the given input u.

The step 1) leverages a key property of the 6DoF-SVC (IN1) model described in [2]: under stationary input conditions, the internal states including perceived motion, converge to "appropriate" values. If the correct initial state x(0) is already known, step 1) may be skipped.

Note that step (1) is particularly useful when applying the 6DoF-SVC model to real-world IMU data. For example, it is effective when the IMU is not aligned with the intended coordinate system (e.g., due to tilted sensor mounting).

Reference
[1] Inoue, S., Liu, H., and Wada, T., "Revisiting Motion Sickness Models Based on SVC Theory Considering Motion Perception," SAE Technical Paper 2023-01-0176, 2023, https://doi.org/10.4271/2023-01-0176.

[2] Wada, T., & Bos, J. E. (2025). Theoretical Considerations on Models of Vestibular Self-motion Perception as Inherent in Computational Frameworks of Motion Sickness. Biological Cybernetics.
