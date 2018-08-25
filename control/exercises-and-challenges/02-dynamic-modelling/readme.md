# Spring-Mass-Damper Identification Challenge

![Figure of a sideways spring-mass-damper system](http://ctms.engin.umich.edu/CTMS/Content/Introduction/System/Modeling/figures/mass_spring_damper.png)

Given the SMD system above, you must:

1. Find the system's transfer function G(s)
2. Discretize G(s)
3. Use your discretized model to approximate G(s) through identification
4. Define the SMD system's constructive parameters (i.e. mass, dampening coefficient, spring constant)

The step input (u = 1) and SMD system output data are provided here for a specific set of constructive parameters, recorded with null initial conditions and a sampling time (T) of 0.1 seconds.

If you want to know whether you have identified the correct system (i.e., if you found the correct parameters), send your parameters over to ``` renan.maidana@acad.pucrs.br ``` with the tag "[IMR-CHALLENGE]".

#### Extra

If you would like to experiment more, consider the following:

* How does the sampling time affect the quality of the model approximation?
* What is the effect of the discretization method chosen?
* What is the effect of the approximation order on the model precision?
* What is the effect of the approximation order on the constructive parameters?

# Good luck!