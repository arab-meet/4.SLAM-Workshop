# GraphSLAM(Cartographer)

Author: zaynap, Abdelrahman Eldesouky

## Introduction

GraphSLAM is an algorithm designed to solve the full SLAM problem, similar to FastSLAM. It recovers  **all robot poses and the entire map** , unlike online SLAM, which only recovers the **most recent pose and map**.

![1726602151480](image/GraphSLAM(Cartographer)/1726602151480.png)

GraphSLAM relies on dependencies between current and previous poses, improving upon the accuracy of FastSLAM. In large environments, FastSLAM may struggle because it uses a finite number of particles, and at certain points, there may not be a particle at the most likely location. FastSLAM works with bits of information, but GraphSLAM significantly improves this by using a more robust method for handling pose and map estimation.

![1726601824967](image/GraphSLAM(Cartographer)/1726601824967.png)

## Graphs and GraphSLAM

> TODO: introduction about graphs (nodes, edges, weighted edges, ...)

As the name suggests, GraphSLAM uses **graphs** to represent the robot's poses and its environment. Each pose or feature is represented as  **nodes** , and the connections between them are represented by **edges (constraints)** :

* **Motion constraints** : Represented by a solid line between two robot poses.
* **Measurement constraints** : Represented by a dashed line between a robot pose and an environmental feature.

As the robot moves, new nodes are added to the graph, expanding it as more information is gathered.

![1726602462897](image/GraphSLAM(Cartographer)/1726602462897.png)

## Front-End vs. Back-End in GraphSLAM

The goal of GraphSLAM is to construct a **graph** that represents all the robot's poses and environmental features, then use this graph to find the **most likely robot path** and **accurate map** of the environment. To achieve this, GraphSLAM is divided into two major components: the **front-end** and the  **back-end**.

![1726602506237](image/GraphSLAM(Cartographer)/1726602506237.png)

### Front-End

The **front-end** of GraphSLAM is responsible for constructing the graph by processing the odometry and sensor data collected by the robot. This involves several key steps:

* **Interpreting sensor data** to identify landmarks and extract relevant features from the environment.
* **Creating and updating the graph** , where each node represents a robot pose or environmental feature.
* **Adding nodes and edges** , ensuring that motion and measurement constraints are accurately represented.

The front-end is highly application-dependent, meaning its performance and design can vary based on:

* **Accuracy requirements** for the map and localization.
* **Type of sensors** being used (e.g., lidar, cameras, or sonar).
* **Environmental factors** , such as indoor vs. outdoor environments or static vs. dynamic conditions.

A significant challenge in the front-end is solving the **data association** problem, which determines whether a detected feature has been previously observed. Correctly associating new sensor data with existing features is critical for accurate mapping

The main challenge in frontend is to assign the same landmark id to all the matched features.

![1726604194299](image/GraphSLAM(Cartographer)/1726604194299.png)

### Back-End

The **back-end** of GraphSLAM takes the complete graph from the front-end, which includes all the robot poses and environmental features connected by constraints, and performs  **graph optimization** . The goal of the back-end is to find the most likely configuration of robot poses and environmental features, minimizing the error in the graph.

This optimization process involves:

* **Minimizing constraint errors** to produce a consistent map and trajectory.
* Solving the graph using techniques like **Maximum Likelihood Estimation (MLE)** to ensure the solution is as accurate as possible.

![1726604212031](image/GraphSLAM(Cartographer)/1726604212031.png)

Graph optimization is central to the back-end, as it refines the entire graph to ensure consistency across all nodes and edges, ultimately leading to the most probable estimate of the robot’s path and the map of its environment.

![1726604231049](image/GraphSLAM(Cartographer)/1726604231049.png)

> The front-end and back-end of GraphSLAM can be completed in two ways:
>
> **1. In order (sequentially)**.
>
> **2. Iteratively** , refining the results over time.

## Graph Optimization

The core of GraphSLAM lies in  **graph optimization**, ensuring that the errors in constraints are minimized to provide the most accurate representation of the robot's path and the environment. GraphSLAM offers a robust and scalable solution to the full SLAM problem by applying optimization techniques such as MLE, particularly in large-scale environments where traditional particle-based approaches like FastSLAM may struggle.

![1726602747414](image/GraphSLAM(Cartographer)/1726602747414.png)

### Maximum Likelihood Estimation (MLE)

The **Maximum Likelihood Estimation (MLE)** method is crucial in solving this optimization problem. It enables us to estimate the most probable configuration of robot poses and environmental features, based on noisy motion and sensor measurements.

#### Likelihood

To understand MLE, it's essential to differentiate between **probability** and  **likelihood** :

* **Probability** is the likelihood of obtaining a particular outcome, given known parameters.

  ![1726602987338](image/GraphSLAM(Cartographer)/1726602987338.png)
* **Likelihood** , on the other hand, is the reverse: estimating which parameters are most likely to explain observed outcomes.

  ![1726602996327](image/GraphSLAM(Cartographer)/1726602996327.png)

In the context of SLAM, MLE is used to compute the **most likely configuration of robot poses and map features** that best explain the observed data (i.e., motion and sensor readings). Since these observations are subject to noise, MLE helps minimize the discrepancies between the predicted and actual measurements.

#### Feature Measurement Example

To better understand MLE in GraphSLAM, consider the case where the robot collects several measurements of a feature's location. Let's focus on a simple case where the robot takes two noisy measurements of the same feature. These measurements provide conflicting information, and MLE is used to find the most likely location of the feature.

Suppose a robot takes two measurements of a feature’s distance from its current position:

* The first measurement is `z_1 = 1.6 m`.
* The second measurement is `z_2 = 2.4 m`.

Given these conflicting measurements, the system is overdetermined, meaning there are more equations than unknowns. To resolve this, we use MLE to find the most likely location of the feature by minimizing the total error.

For the first measurement, we have:

$$
p(x) = \frac{1}{\sigma \sqrt{2\pi}} \exp\left( -\frac{1}{2} \frac{(z_1 - (x_0 + 1.6))^2}{\sigma^2} \right)
$$

Where:

* `x` is the feature's true location.
* `z_1` is the measurement.
* `σ^2` is the variance of the noise associated with the measurement.

For the second measurement `z_2`, the likelihood function is:

$$
p(x) = \frac{1}{\sigma \sqrt{2 \pi}} \exp \left(-\frac{1}{2} \frac{(z_2 - 2.4)^2}{\sigma^2}\right)
$$

**Maximizing the Likelihood**

In MLE, we aim to find the feature location xx**x** that **maximizes the likelihood** of the observed measurements. Since the measurements are independent, the overall likelihood is the product of the likelihoods of the individual measurements:

$$
p(x) = \left( \frac{1}{\sigma \sqrt{2 \pi}} \exp \left(-\frac{1}{2} \frac{(z_1 - 1.6)^2}{\sigma^2}\right) \right) \cdot \left( \frac{1}{\sigma \sqrt{2 \pi}} \exp \left(-\frac{1}{2} \frac{(z_2 - 2.4)^2}{\sigma^2}\right) \right)
$$

To find the maximum likelihood estimate, follow these steps:**Remove Scaling Factors**

1. The value of m that maximizes the equation does not depend on the constants in front of each of the exponentials. These are *scaling factors*, so in SLAM we are not usually interested in the absolute value of the probabilities, but in finding the maximum likelihood estimate. For this reason, the factors can be removed.

   $$
   p(x) = \exp \left(-\frac{1}{2} \frac{(z_1 - 1.6)^2}{\sigma^2}\right)  \cdot \exp \left(-\frac{1}{2} \frac{(z_2 - 2.4)^2}{\sigma^2}\right)
   $$
2. **Log-Likelihood**

   The product of the probabilities has been simplified, but the equation is still rather complicated - with exponentials present. First, we must combining the two exponentials into one.

   $$
   p(x) = \exp \left(-\frac{1}{2} \frac{(z_1 - 1.6)^2}{\sigma^2}-\frac{1}{2} \frac{(z_2 - 2.4)^2}{\sigma^2}\right)
   $$

   However, it's more convenient to work with the logarithm of the likelihood function (log-likelihood), which transforms the product of exponentials into a sum:

   $$
   \ln p(x) =-\frac{1}{2} \frac{(z_1 - 1.6)^2}{\sigma^2}-\frac{1}{2} \frac{(z_2 - 2.4)^2}{\sigma^2}
   $$

   > One thing to note when working with logs of likelihoods, is that they are always negative. This is because probabilities assume values between 0 and 1, and the log    of any value between 0 and 1 is negative. For this reason, when working with log-likelihoods, optimization entails minimizing the negative log-likelihood; whereas in    the past, we were trying to maximize the likelihood.
   >

   **as was done before**, the constants in front of the equation can be removed without consequence. As well, for the purpose of this example, we will assume that the same sensor was used in obtaining both measurements - and will thus ignore the variance in the equation.

   $$
   (z_1 - 1.6)^2-(z_2 - 2.4)^2
   $$
3. **Optimization**
   The equation has been reduced greatly. To get it to its simplest form, since the measuerments `z_1` and `z_2` for the same feature, then the equation will be:

   $$
   (z - 1.6)^2-(z - 2.4)^2
   $$

   This can be simplified to:

   $$
   2z^2-8+8.32
   $$

   To find the minimum of this equation, you can take the first derivative of the equation and set it to equal 0.

   $$
   \frac{d}{dx}(2z^2-8+8.32)=4z-8=0
   $$

   $$
   4z=8
   $$

   $$
   z=2
   $$

#### Applying MLE Analytical Solution to GraphSLAM

The last procedure that we executed here is the *analytical* solution to an MLE problem. The steps includes:

* Removing inconsequential constants.
* Converting the equation from one of *likelihood estimation* to one of  *negative log-likelihood estimation.*
* Calculating the first derivative of the function and setting it equal to zero to find the extrema.

In GraphSLAM, the first two steps can be applied to *every* constraint. Thus, any measurement or motion constraint can simply be labelled with its negative log-likelihood error.

1. **Motion constraints** between consecutive robot poses, representing the motion model.
2. **Measurement constraints** between robot poses and observed features, representing the sensor model.

For each constraint, we model the error between the predicted and observed values as a Gaussian distribution. The total error across all constraints in the system is expressed as:

$$
J_{\text{GraphSLAM}} = \sum_t \frac{(x_t - (x_{t-1} + u_t))^2}{\sigma_u^2} + \sum_t \frac{(z_t - (x_t + m_t))^2}{\sigma_z^2}
$$

Where:

* `x_t`: The robot's pose at time `t`.
* `u_t`: The motion command (e.g., velocity, odometry) between two poses.
* `z_t`: The measurement of a feature from pose `x_t`.
* `m_t`: The location of the feature being observed.
* `σ_u^2` `σ_z^2`: Variances associated with the motion and measurement noise, respectively.

MLE plays a vital role in GraphSLAM by helping resolve noisy measurements and compute the most probable configuration of robot poses and feature locations. By minimizing the sum of squared errors between predicted and observed values, MLE provides a robust framework for solving the simultaneous localization and mapping problem.

########### 

Review :

![1715037052893](image/GraphSLAM(Cartographer)/1715037052893.png)

# graph slam

![1715036819526](image/GraphSLAM(Cartographer)/1715036819526.png)

use the graph theory and optmization algorithims

# why

brief history of graph slam

![1715036835037](image/GraphSLAM(Cartographer)/1715036835037.png)

![1715036852878](image/GraphSLAM(Cartographer)/1715036852878.png)

## graph based VS filter based

## benefites

## when to choose graph slam

### goal

The goal is to create a map by estimation of the robot’s state x(t) using measurements z(t) and control inputs u(t).

![1715036866337](image/GraphSLAM(Cartographer)/1715036866337.png)

# graph slam concepts

## graph construction

### main concept :

* nodes(poses )
* edges (constraints)
* information matrix
* linear graph without ever returning to a previously visited location
* cyclical graph with revisiting a location has been to before after some timer has passed

In such a case, features in the environment will be linked to multiple poses — ones that are not consecutive but spaced far apart.

### nodes

#### types of nodes

* initial pose
* standard pose

### constraints (edges encoding the spatial relationship between two posetions)

* initial location constraint
* pose-pose constraints
* pose-landmark constraints

![1715036907278](image/GraphSLAM(Cartographer)/1715036907278.png)

![1715036968107](image/GraphSLAM(Cartographer)/1715036968107.png)

![1715036983809](image/GraphSLAM(Cartographer)/1715036983809.png)

### information matrix

## filtering (frontend processing)

## smoothing or optmization (backend )

### the different frames range in both

### loop closure

### linearization

### iterative optmization

### handle dynamic environment

### large scale slam (effiecint graph mangment)

### multi robot slam

## graph optmization

# [Next Topic Link]

# References:

### [&lt;-Back to main](../../README.md)
