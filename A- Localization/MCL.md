# Monte Carlo Localization (MCL):

### 1.  **MCL Overview**

<p style="text-align: center;"><img src="images/MCL3.gif" width="75%" />

- Monte Carlo Localization (MCL) is a probabilistic algorithm used in robotics to estimate the position and orientation (pose) of a mobile robot within a known environment

* MCL represents the robot's belief about its position using a set of particles, each of which represents a possible location and orientation (pose) of the robot. These particles are sampled across the robot’s environment.
* Each particle has a weight, representing how likely that particular pose is to be correct based on the robot’s sensor data.

<p style="text-align: center;"><img src="images/MCL.png"  width="40%"/> <p style="text-align: center;"><img src="images/MCL2.png"  width="40%"/>

### **2. Problem that MCL will solve**

Monte Carlo Localization (MCL) can solve both **local localization** and **global localization** problems

1. **Local Localization:**

* **Problem** : Local localization assumes that the robot already has a rough idea of its initial position within a known environment and only needs to correct small errors due to noise or movement.
* **MCL Solution** : In this case, MCL is initialized with a small set of particles concentrated around the robot's known initial position. As the robot moves and gathers sensor data, MCL adjusts the particles to refine its estimate of the pose.

2. **Global Localization:**

* **Problem** : In global localization, the robot has no prior knowledge of its initial position, or it might be placed in an entirely unknown location within a known map. The robot must figure out where it is from scratch.
* **MCL Solution** : To solve global localization, MCL initializes a large number of particles randomly distributed across the entire map. As the robot moves and collects sensor data, it compares sensor readings with the map and gradually eliminates unlikely positions, converging on the correct location.

### **3. Particle Filter**

- The key idea of MCL is to represent the belief by a set of samples (also called:
  particles), drawn according to the posterior distribution over robot poses.
- A **particle filter** is a type of **Bayesian filtering** technique that approximates the probability distribution of a system’s state (in this case, the robot's pose) using a set of particles. Each particle represents a potential state (a hypothesis) of the robot’s position, and the algorithm updates the particles over time based on the robot’s movements and sensor data.
- **Bayesian filtering** is a probabilistic approach used to estimate the state of a dynamic system over time, using noisy observations and prior knowledge about the system.
- Each particle includes both the robot’s position and orientation, along with a weight that reflects how likely this hypothesis is to be correct based on sensor data.The standard state of each particle is represented as:

* **x** : The x-coordinate of the robot’s position in a 2D plane.
* **y** : The y-coordinate of the robot’s position in the same 2D plane.
* **θ (theta)** : The orientation (or heading) of the robot, typically measured in radians or degrees, which represents the direction the robot is facing.
* **w** : The weight of the particle, which represents the likelihood that this particle's state (x, y, θ) corresponds to the robot’s actual pose.

<p style="text-align: center;"><p style="text-align: center;"><img src="images/MCL4.png" width="75%"/>

- particles are initially spread randomly and uniformly throughout this entire map.
- Particles with large weights are more likely to survive during a resampling process.
  survive whereas the others are more likely to die.
- Finally, after several iterations of the Monte Carlo localization algorithm,and after different stages of resampling,particles will converge and estimate the robot's pose.

<p style="text-align: center;"><p style="text-align: center;"><img src="images/MCL5.png"  width="75%" >

### **4.MCL vs KF**

<p style="text-align: center;"><p style="text-align: center;"><img src="images/MCL3.png" />

1. **Measurements:** MCL can use any type of raw measurement, while EKF requires landmarks or requires measurements to be related to specific landmarks or features.
2. **Measurement Noise:** MCL can handle any type of noise, while EKF assumes Gaussian noise, EKF relies on the assumption that noise is normally distributed, which can limit its effectiveness in scenarios where this assumption does not hold. This allows MCL to model a much greater variety of environments especially since you can't always model the real world with Gaussian distributions.

<p style="text-align: center;"><p style="text-align: center;"><img src="images/Gaussian.png"  width="40%" >

3. **Posterior:**

   - MCL represents  the posterior distribution as a set of particles. Each particle represents a potential state of the robot, and the distribution of particles approximates the robot’s belief about its position and orientation.
   - **EKF** : Assumes that the posterior distribution is Gaussian. EKF represents the belief about the robot’s state using a mean and a covariance matrix, which simplifies calculations but can be limiting if the true distribution is not Gaussian.
   - The posterior distribution ( P(x_t | z_{1:t}) ) represents the probability of the system being in state ( x_t ) at time ( t ), given all observations ( z_{1:t} ) up to and including time ( t ).
   - so if the robot navigating a cluttered room. If the robot is uncertain about its exact position and there are obstacles that cause significant deviations in its measurements:
     - **Gaussian Distribution** : If the robot’s possible positions were Gaussian, its uncertainty would be symmetrically spread around a central estimate.
     - **Non-Gaussian Distribution** : In reality, the robot’s position might be uncertain due to multiple possible obstacles, resulting in a multi-modal distribution (several peaks where the robot might be). so the EKF's approximation might be inaccurate.
4. **Efficiency:** Both MCL and EKF are generally efficient in terms of memory and time.
5. **Ease of Implementation:** EKF is slightly more complex to implement than MCL.
6. **Robustness:**

* MCL is more robust to noise and outliers than EKF.

- EKF is Less robust to noise and outliers due to its reliance on Gaussian noise assumptions. Significant deviations or outliers can lead to inaccurate state estimates

7. **Memory & Resolution Control:**

- MCL Allows more control over memory and resolution by adjusting the number of particles. Increasing the number of particles improves resolution but requires more memory and computational power.

8. **Global Localization:** MCL can handle global and local localization, while EKF is limited to local localization.

### **5. Bayes Filter**

The MCL algorithm estimates the posterior distribution of a robot’s position and orientation based on sensory information. This process is known as a recursive `Bayes filter`.

Using a Bayes filtering approach, roboticists can estimate the **state** of a **dynamical system** from sensor  **measurements** .

The goal of Bayes filtering is to estimate a probability density over the state space conditioned on the measurements. The probability density, or also known as **posterior** is called the **belief** and is denoted as:

<p style="text-align: center;"><p style="text-align: center;"><img src="images/Posterior.png"  width="40%" >

### **6.MCL Algorithm Explaination and Psedu Code**

<p style="text-align: center;"><p style="text-align: center;"><img src="images/MCL_Algorithm.png" >

* The Monte Carlo Localization Algorithm is composed of two main sections represented by two for loops.
* The first section is the motion and sensor update, and the second one is the resampling process.
* Given a map of the environment, the goal of the MCL is to determine the robot's pose represented by the belief.
* At each iteration, the algorithm takes the previous belief, the actuation command, and the sensor measurements as input.
* Initially, the belief is obtained by randomly generating m particles.
* Then, in the first for loop, the hypothetical state is computed whenever the robot moves.
* Following, the particles weight is computed using the latest sensor measurements.
  Now, motion and measurement are both added to the previous state.
* Moving onto the second section of the MCL where a sampling process happens.  
* Here, the particles with high probability survive and are re-drawn in the next iteration, while the others die.Finally, the algorithm outputs the new belief and another cycle of iteration starts implementing the next motion by reading the new sensor measurements

### **7. The AMCL package ROS**

The AMCL (Adaptive Monte Carlo Localization) package provides the amcl node, which uses the
MCL system in order to track the localization of a robot moving in a 2D space. This node
subscribes to the data of the laser, the laser-based map, and the transformations of the
robot, and publishes its estimated position in the map. On startup, the amcl node initializes its
particle filter according to the parameters provided.

**Note:** the word Adaptive has been added to the Monte Carlo Localization algorithm. This is because, in this node, we will be able to configure (adapt) some of the parameters that are used in this algorithm.

**Link** : https://wiki.ros.org/amcl

### **8. AMCL Tunning**

[-Back to main](../README.md)
