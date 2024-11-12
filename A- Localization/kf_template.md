# Kalman Filter

## 1. **Introduction**

Imagine you're trying to track a moving car using a GPS device. However, the GPS readings are not perfect—they have errors and noise. Additionally, you might know that the car generally moves at a steady speed but occasionally accelerates or slows down unpredictably.

How can you combine these noisy GPS measurements with your understanding of the car's motion to estimate its true position and speed accurately?

The **Kalman Filter** is a mathematical tool that helps solve this problem. It provides a way to optimally estimate the true state (like position and velocity) of a dynamic system by combining measurements and a model, even when both are imperfect.

---

## 2. **Fundamental Concepts**

Before diving into the Kalman Filter, let's revisit some key statistical concepts that are foundational to understanding how it works.

### **2.1. Mean $(\mu)$ and Expected Value $( E[X] )$**

- **Mean $(\mu )$**: The average of a set of numbers, calculated by summing all the values and dividing by the number of values.
- **Expected Value $(E[X])$**: The theoretical mean of a random variable's probability distribution, representing the average outcome if an experiment is repeated infinitely many times.

**Example**:

Suppose you have five coins: two worth 5 cents and three worth 10 cents.

$$
\mu = \frac{1}{N} \sum _{n=1}^{N}V_{n}=\frac{5 + 5 + 10 + 10 + 10}{5} = \frac{40}{5} = 8 \text{ cents}
$$

Now, imagine you measure a person's weight five times due to slight variations in the scale:

- 79.8 kg
- 80.0 kg
- 80.1 kg
- 79.8 kg
- 80.2 kg

The estimated true weight is:

$$
E[w] =\frac{1}{N} \sum _{n=1}^{N}W_{n}= \frac{79.8 + 80.0 + 80.1 + 79.8 + 80.2}{5} = 79.98 \text{ kg}
$$

### **2.2. Variance $( \sigma^2)$ and Standard Deviation $(\sigma )$**

- **Variance $(\sigma^2 )$**: Measures how much the values in a dataset spread out from the mean.
- **Standard Deviation $(\sigma)$**: The square root of the variance, representing the average amount by which each value differs from the mean.

**Example**:

Consider the heights of players on two basketball teams:

**Team A**:

- Heights $(m)$: 1.89, 2.10, 1.75, 1.98, 1.85
- Mean height $(\mu )$: 1.914 m

**Team B**:

- Heights $(m)$: 1.94, 1.90, 1.97, 1.89, 1.87
- Mean height $(\mu)$: 1.914 m

Both teams have the same mean height, but Team A has a higher variance, indicating more diversity in player heights.

**Calculating Variance for Team A**:

1. Subtract the mean from each height and square the result.
2. Sum these squared differences.
3. Divide by the number of players.

$$
\sigma_{A}^2 = \frac{1}{N} \sum _{n=1}^{N} \left( x_{A_n}-  \mu  \right) ^{2}= \frac{(1.89 - 1.914)^2 + \dots + (1.85 - 1.914)^2}{5}=0.014m^{2}
$$

**Variance for Team B**:

$$
\sigma_{B}^2 = \frac{1}{N} \sum _{n=1}^{N} \left( x_{B_n}-  \mu  \right) ^{2}= \frac{(1.94 - 1.914)^2 + \dots + (1.87 - 1.914)^2}{5}=0.0013m^{2}
$$

**Calculating the standard deviation for Team A**:

1. take the square root of the variance

$$
\sigma_{A} =\sqrt{\frac{1}{N} \sum _{n=1}^{N} \left( x_{A_n}-  \mu  \right) ^{2}}=\sqrt{0.014}=0.12m
$$

**Standard deviation for Team B**:

$$
\sigma_{B} =\sqrt{\frac{1}{N} \sum _{n=1}^{N} \left( x_{B_n}-  \mu  \right) ^{2}}=\sqrt{0.0013}=0.036m
$$

### **2.3. Normal Distribution (Gaussian Distribution)**

A normal distribution is a bell-shaped curve that is symmetric about the mean. It's defined by its mean $(\mu )$ and variance $( \sigma^2)$.

**Probability Density Function**:

$$
f(x; \mu, \sigma^2) = \frac{1}{\sqrt{2\pi \sigma^2}} e^{- \frac{(x - \mu)^2}{2\sigma^2}}
$$

**Properties**:

- Approximately 68% of values fall within one standard deviation of the mean.
- Approximately 95% fall within two standard deviations.
- Approximately 99.7% fall within three standard deviations.
  ![gaussian distribution](images/Gaussian_Distribution_3.png)

### **2.4. Random Variables**

A random variable represents a numerical outcome of a random process. In our context, measurements and system states are treated as random variables due to inherent uncertainties.

### **2.5. Estimate, Accuracy, and Precision**

- **Estimate**: An approximation of a true value based on observations.
- **Accuracy**: How close an estimate is to the actual true value.
- **Precision**: The consistency of repeated measurements.

**Example**:

- **High Accuracy, High Precision**: Darts are clustered at the bullseye.
- **High Precision, Low Accuracy**: Darts are clustered together but away from the bullseye.
- **Low Precision, High Accuracy**: Darts are spread out but centered around the bullseye.
- **Low Precision, Low Accuracy**: Darts are spread out and away from the bullseye.
  ![AccuracyAndPrecision](images/AccuracyAndPrecision.png)

in summery
----------

A measurement is a **random variable**, described by the **Probability Density Function (PDF)**.

The mean of the measurements is the **Expected Value** of the random variable.

The offset between the mean of the measurements and the true value is the **accuracy of the measurements**, also known as bias or **systematic measurement error**.

The dispersion of the distribution is the measurement **precision, also known as the **measurement noise, random measurement error**, or **measurement uncertainty**.
![statistical_view](images/statistical_view.png)

---

## 3. **Understanding the Kalman Filter**

### **3.1. What is the Kalman Filter?**

The Kalman Filter is an algorithm that optimally estimates the state of a dynamic system by minimizing the mean of the squared errors. It does this by combining predictions from a model with new measurements, updating estimates in a recursive manner.

### **3.2. Why Use the Kalman Filter?**

- To **filter out noise** from measurements.
- To **predict future states** based on a model.
- To **estimate** values that are not directly measurable (e.g., velocity from position measurements).

### **3.3. How Does the Kalman Filter Work?**

The Kalman Filter operates in two main steps:

1. **Prediction Step**: Estimates the current state and uncertainty from the previous state.
2. **Update Step**: Corrects the prediction using the new measurement.

These steps are repeated recursively with each new measurement.
![Kalman filter process](images/Kalman_filter_process_1.png)

## 4. **Mathematical Foundations**

### **4.1. State Representation**

The system's state at any time $ k $ is represented by a vector $ \mathbf{x}_k $.

**Example (1D motion)**:

$\mathbf{x}_k = \begin{bmatrix} x_k \\ \dot{x}_k \end{bmatrix}$

- $ x_k $: Position at time $ k $.
- $\dot{x}_k $: Velocity at time $ k $.

### **4.2. Process Model**

Describes how the state evolves over time.

**State Transition Equation**:

$$
\mathbf{x}_k = \mathbf{A}\mathbf{x}_{k-1} + \mathbf{B}\mathbf{u}_k + \mathbf{w}_k
$$

- $ \mathbf{A} $: State transition matrix.
- $ \mathbf{B} $: Control input matrix.
- $ \mathbf{u}_k $: Control input at time $ k $.
- $ \mathbf{w}_k $: Process noise (zero-mean Gaussian with covariance $ \mathbf{Q} $).

**Example (constant velocity model)**:

$$
\mathbf{A} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
$$

### **4.3. Measurement Model**

Relates the state to the measurements.

**Measurement Equation**:

$$
\mathbf{z}_k = \mathbf{H}\mathbf{x}_k + \mathbf{v}_k
$$

- $ \mathbf{z}_k $: Measurement at time $ k $.
- $ \mathbf{H} $: Measurement matrix.
- $ \mathbf{v}_k $: Measurement noise (zero-mean Gaussian with covariance $ \mathbf{R} $).

**Example**:

If we only measure position:

$$
\mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}
$$

### **4.4. Covariance Matrices**

- **Process Noise Covariance $( \mathbf{Q} )$**: Uncertainty in the process model.
- **Measurement Noise Covariance $( \mathbf{R} )$**: Uncertainty in the measurements.
- **Error Covariance $( \mathbf{P}_k )$**: Uncertainty in the state estimate.

### **4.5. Kalman Gain $(\mathbf{k} )$**

Determines how much the new measurement influences the state estimate.

---

## 5. **The Kalman Filter Algorithm**

At each time step $ k $, perform the following steps:

### **5.1. Prediction Step**

- **State Prediction**:

  $$
  \mathbf{\hat{x}}_{k|k-1} = \mathbf{A}\mathbf{\hat{x}}_{k-1|k-1} + \mathbf{B}\mathbf{u}_k
  $$
- **Error Covariance Prediction**:

  $$
  \mathbf{P}_{k|k-1} = \mathbf{A}\mathbf{P}_{k-1|k-1}\mathbf{A}^T + \mathbf{Q}
  $$

### **5.2. Update Step**

- **Kalman Gain Calculation**:

  $$
  \mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T (\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R})^{-1}
  $$
- **State Update**:

  $$
  \mathbf{\hat{x}}_{k|k} = \mathbf{\hat{x}}_{k|k-1} + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}\mathbf{\hat{x}}_{k|k-1})
  $$
- **Error Covariance Update**:

  $$
  \mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
  $$

---

## 6. **Examples**

### **6.1. Simple 1D Example**

**Problem**:

Estimate the temperature of a room using noisy thermometer readings.

- **Initial Estimate**: $\hat{T}_0 = 20^\circ \text{C}$
- **Initial Estimate Variance**:$P_0 = 4^\circ \text{C}^2 $
- **Measurement Variance**: $R = 9^\circ \text{C}^2 $
- **Measurements**: $ z_1 = 23^\circ \text{C}, z_2 = 21^\circ \text{C}, z_3 = 22^\circ \text{C} $

**Solution**:

Assume no process noise $( Q = 0 )$ and no control input $( u = 0 )$.

**Time Step 1**:

- **Prediction**:

  $$
  \hat{T}_{1|0} = \hat{T}_0 = 20^\circ \text{C}\\ ~\\
  P_{1|0} = P_0 = 4^\circ \text{C}^2
  $$
- **Kalman Gain**:

$$
K_1 = \frac{P_{1|0}}{P_{1|0} + R} = \frac{4}{4 + 9} = 0.3077
$$

- **Update**:

  $$
  \hat{T}_{1|1} = \hat{T}_{1|0} + K_1(z_1 - \hat{T}_{1|0}) = 20 + 0.3077 \times 3 = 20.9231^\circ \text{C} \\ ~\\
  P_{1|1} = (1 - K_1) P_{1|0} = 0.6923 \times 4 = 2.7692^\circ \text{C}^2
  $$

Repeat these steps for $ z_2 $ and $ z_3 $.

### **6.2. 2D Tracking Example**

**Problem**:

Track an object's position and velocity in 2D space with noisy position measurements.

**Setup**:

- **State Vector**:

  $$
  \mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ y \\ \dot{y} \end{bmatrix}
  $$
- **State Transition Matrix**:

  $$
  \mathbf{A} = \begin{bmatrix} 1 & \Delta t & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & \Delta t \\ 0 & 0 & 0 & 1 \end{bmatrix}
  $$
- **Measurement Matrix**:

  $$
  \mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}
  $$
- **Process Noise Covariance**:

  $$
  \mathbf{Q} = q \times \mathbf{I}_{4 \times 4}
  $$
- **Measurement Noise Covariance**:

  $$
  \mathbf{R} = \sigma^2 \times \mathbf{I}_{2 \times 2}
  $$

**Solution**:

At each time step:

1. **Prediction**:

   $$
   \mathbf{\hat{x}}_{k|k-1} = \mathbf{A}\mathbf{\hat{x}}_{k-1|k-1} \\ ~\\
   \mathbf{P}_{k|k-1} = \mathbf{A}\mathbf{P}_{k-1|k-1}\mathbf{A}^T + \mathbf{Q}
   $$
2. **Update**:

   $$
   \mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^T (\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R})^{-1}\\ ~\\
   \mathbf{\hat{x}}_{k|k} = \mathbf{\hat{x}}_{k|k-1} + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}\mathbf{\hat{x}}_{k|k-1})\\ ~\\
   \mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
   $$

**Interpretation**:

The filter estimates both position and velocity, even though only position is measured.

---

## 7. **Comparison with Other Filters**

### **7.1. Kalman Filter vs. Particle Filter**

- **Kalman Filter**:

  - Assumes linearity and Gaussian noise.
  - Uses mean and covariance to represent the state.
  - Computationally efficient.
- **Particle Filter**:

  - Handles non-linearities and non-Gaussian noise.
  - Represents the state with a set of particles.
  - Computationally intensive.

### **7.2. Advantages of the Kalman Filter**

- **Optimality**: Provides the best estimate under certain conditions.
- **Efficiency**: Suitable for real-time applications.
- **Simplicity**: Easier to implement for linear systems.

### **7.3. Limitations**

- **Linearity**: Assumes system is linear.
- **Gaussian Noise**: Assumes process and measurement noise are Gaussian.
- **Model Accuracy**: Requires accurate system models.

---

## 8. **Applications**

- **Navigation**: GPS and inertial navigation systems.
- **Robotics**: Localization and sensor fusion.
- **Economics**: Filtering and predicting economic indicators.
- **Signal Processing**: Noise reduction and tracking.
- **Control Systems**: Estimating states for feedback control.

---

## **References and Further Reading**

- **Books**:

  - *"Kalman Filtering: Theory and Practice"* by Mohinder S. Grewal and Angus P. Andrews.
  - *"Optimal State Estimation"* by Dan Simon.
- **Tutorials**:

  - *"An Introduction to the Kalman Filter"* by Greg Welch and Gary Bishop.
- **Online Resources**:

  - *"How a Kalman Filter Works, in Pictures"* by Matt B. (For intuitive understanding).

---
