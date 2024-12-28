Author: Rawda Abdelkhalek 

Review : KG


# Kalman Filter

## 1. **Introduction**

Imagine you're trying to track a moving car using a GPS device. However, the GPS readings are not perfect — they have errors and noise. Additionally, you might know that the car generally moves at a steady speed but occasionally accelerates or slows down unpredictably.

How can you combine these noisy GPS measurements with your understanding of the car's motion to estimate its true position and speed accurately?

The **Kalman Filter** is a mathematical tool that helps solve this problem. It provides a way to optimally estimate the true state (like position and velocity) of a dynamic system by combining measurements and a model, even when both are imperfect.

<p align="center">
  <img src="images/Kf_intro.jpg">
</p>

## 2. **Fundamental Concepts**

Before diving into the Kalman Filter, let's revisit some key statistical concepts that are foundational to understanding how it works.

### **2.1. Mean $(\mu)$ and Expected Value $( E[X] )$**

- **Mean $(\mu )$**: The average of a set of numbers, calculated by summing all the values and dividing by the number of values.
- **Expected Value $(E[X])$**: The theoretical mean of a random variable's probability distribution, representing the average outcome if an experiment is repeated infinitely many times.

**Example**:

Suppose you have five coins: two worth 5 cents and three worth 10 cents.

$$
\mu = \frac{1}{N} \sum _ {n=1}^{N}V _{n}=\frac{5 + 5 + 10 + 10 + 10}{5} = \frac{40}{5} = 8 \text{ cents}
$$

Now, imagine you measure a person's weight five times due to slight variations in the scale:

- 79.8 kg
- 80.0 kg
- 80.1 kg
- 79.8 kg
- 80.2 kg

The estimated true weight is:

$$
E[w] =\frac{1}{N} \sum _ {n=1}^{N}W _{n}= \frac{79.8 + 80.0 + 80.1 + 79.8 + 80.2}{5} = 79.98 \text{ kg}
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
\sigma_{A}^2 = \frac{1}{N} \sum _ {n=1}^{N} \left( x _{A_n}-  \mu  \right) ^{2}= \frac{(1.89 - 1.914)^2 + \dots + (1.85 - 1.914)^2}{5}=0.014m^{2}
$$

**Variance for Team B**:

$$
\sigma_{B}^2 = \frac{1}{N} \sum _ {n=1}^{N} \left( x _{B_n}-  \mu  \right) ^{2}= \frac{(1.94 - 1.914)^2 + \dots + (1.87 - 1.914)^2}{5}=0.0013m^{2}
$$

**Calculating the standard deviation for Team A**:

1. take the square root of the variance

$$
\sigma_{A} =\sqrt{\frac{1}{N} \sum _ {n=1}^{N} \left( x _{A_n}-  \mu  \right) ^{2}}=\sqrt{0.014}=0.12m
$$

**Standard deviation for Team B**:

$$
\sigma_{B} =\sqrt{\frac{1}{N} \sum _ {n=1}^{N} \left( x_ {B_n}-  \mu  \right) ^{2}}=\sqrt{0.0013}=0.036m
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

In summary
----------

A measurement is a **random variable**, described by the **Probability Density Function (PDF)**.

The mean of the measurements is the **Expected Value** of the random variable.

The offset between the mean of the measurements and the true value is the **accuracy of the measurements**, also known as bias or **systematic measurement error**.

The dispersion of the distribution is the measurement **precision, also known as the **measurement noise, random measurement error**, or **measurement uncertainty**.
![statistical_view](images/statistical_view.png)

---

## 3. **Understanding The Kalman Filter**

### **3.1. What is the Kalman Filter?**

The Kalman Filter is an algorithm that optimally estimates the state of a dynamic system by minimizing the mean of the squared errors. It does this by combining predictions from a model with new measurements, updating estimates in a recursive manner.

**3.2. Why Use the Kalman Filter?**

- To **filter out noise** from measurements.
- To **predict future states** based on a model.
- To **estimate** values that are not directly measurable (e.g., velocity from position measurements).

### **3.3. How Does the Kalman Filter Work?**

The Kalman Filter operates in two main steps:

1. **Prediction Step**: Estimates the current state and uncertainty from the previous state.
2. **Update Step**: Corrects the prediction using the new measurement.

These steps are repeated recursively with each new measurement.
![Kalman filter process](images/Kalman_filter_process_2.png)

### **3.4. Kalman Filter Overview**

<img src="images/kf_design1.png" width="100%">

The Kalman Filter algorithm will go through the following steps:

* **first measurement**  :
  the filter will receive initial noisy measurements from the sensor.
* **initialize state and covariance matrices** :
  the filter will initialize the robot's position based on the first measurement.
* then the kalman filter will receive another sensor  measurement after a time period Δt.
* **predict** :
  the algorithm will predict where the robot will be after time Δt.
* **update** :
  the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value. 
### 3.5. **Mathematical Overview**

![kf_over_view.png](images/kf_over_view.png)
## 4. **Mathematical Foundations**

### **4.1. State Representation**

The system's state at any time $ k $ is represented by a vector $\mathbf{X^{'}}_k$

The state vector:

<p align="center">
  <img src="images/state_vector.jpg" width="15%">
</p>


- $ p^{'}_k  $: Position at time $ k $.
- $v^{'}_k$: Velocity at time $ k $.

**Example**:

1. 1D motion:
<p align="center">
  <img src="images/1d_state_vector.jpg
" width="15%">
</p>

2. 2D motion:
<p align="center">
  <img src="images/2d_state_vector.jpg
" width="15%">
</p>

### **4.2. Process Model**

Describes how the state evolves over time.

**State Transition Equation**:
<p align="center">
  <img src="images/State_Transition_Equation.jpg" width="30%">
</p>

- $ \mathbf{A} $: State transition matrix.
- $ \mathbf{B} $: Control input matrix.
- $ \mathbf{U}_k $: Control input at time $ k $.
- $ \mathbf{W}_k $: Process noise (zero-mean Gaussian with covariance $ \mathbf{Q}$).

<p align="center">
  <img src="images/Process_noise.jpg" width="10%">
</p>


**Example 1 (constant velocity model 1-d motion)**:

*The state Vector :*

<p align="center">
  <img src="images/The_state_Vector_ex1.jpg" width="10%">
</p>

*Linear motion :*
<p align="center">
  <img src="images/Linear_motion_ex1.jpg" width="15%">
</p>

*State Transition matrix :*
<p align="center">
  <img src="images/State_Transition_matrix_ex1.jpg" width="30%">
</p>

**Example 2 (constant velocity model 2-d Linear motion):**
<p align="center">
  <img src="images/Process_Model.jpg" width="40%">
</p>

*State Transition Equation*
<p align="center">
  <img src="images/State_Transition_Equation_ex2.jpg" width="30%"> 
</p>
<p align="center">
  <img src="images/State_Transition_Equation_ex2_.jpg" width="20%"> 
</p>

*State Transition matrix :*

<p align="center">
  <img src="images/State_Transition_matrix_ex2 .jpg" width="40%"> 
</p>

**Example 3 (constant acceleration model 2-d motion)**:

<p align="center">
  <img src="images/State_Transition_Equation_ex3.jpg" width="30%"> 
</p>

*Linear motion :*

<p align="center">
  <img src="images/Linear_motion_ex3.jpg" width="30%"> 
</p>

*State Transition matrix :*

<p align="center">
  <img src="images/State_Transition_matrix_ex3.jpg" width="60%"> 
</p>

**Example 4 (constant acceleration model 3-d motion)**:
<p align="center">
  <img src="images/State_Transition_Equation.jpg" width="30%">
</p>

*Linear motion :*
<p align="center">
  <img src="images/Linear_motion_ex4.jpg" width="30%">
</p>

*State Transition matrix :*

<p align="center">
  <img src="images/State_Transition_matrix_ex_4.jpg" width="80%">
</p>

### **4.3. Measurement Model**

Relates the state to the measurements.

**Measurement Equation**:
<p align="center">
  <img src="images/Measurement_Equation.jpg" width="20%">
</p>

- $ \mathbf{Y}_k $: Measurement at time $ k $.
- $ \mathbf{C} $: Measurement matrix.
- $ \mathbf{Z}_k $: Measurement noise (zero-mean Gaussian with covariance $ \mathbf{R} $).

<p align="center">
  <img src="images/Measurement_noise.jpg" width="15%">
</p>

**Example 1 (1d-motion)**:
<p align="center">
  <img src="images/Measurement_Equation.jpg" width="20%">
</p>

If we only measure position:

<p align="center">
  <img src="images/Measurement_matrix.jpg" width="15%">
</p>

*The measurement function:*
<p align="center">
  <img src="images/The_measurement _function_ex1.jpg" width="15%">
</p>

**Example 2 (2d-motion)**:

If we only measure position:
<p align="center">
  <img src="images/Measurement_matrix_ex2.jpg" width="20%">
</p>

*The measurement function:*

<p align="center">
  <img src="images/The_measurement_function_ex2.jpg" width="30%">
</p>

### **4.4. Covariance Matrices**

- **Process Noise Covariance $( \mathbf{Q} )$**: Uncertainty in the process model.
- **Measurement Noise Covariance $( \mathbf{R} )$**: Uncertainty in the measurements.

There is a true state that we want to know the value of but unfortunately we have to measure it with a noisy sensor.

<img src="images/measurement_noise_1.jpg
" width="70%">

- **Error Covariance $( \mathbf{P}_k )$**: Uncertainty in the state estimate.

<img src="images/covariance_matrix.png" width="70%">

### **4.5. Kalman Gain $(\mathbf{k} )$**

Determines how much the new measurement influences the state estimate.

<p align="center">
  <img src="images/Kalman_Gain.jpg" width="80%">
</p>

* note : $0< K<1$
<p align="right">
  <img src="images/0<kg<1.png" width="70%">
</p>

<img src="images/KG.jpg" width="90%">

## 5. **The Kalman Filter Algorithm**

At each time step $ k $, perform the following steps:

### **5.1. Prediction Step**

- **State Prediction**:
<p align="center">
  <img src="images/State_Transition_Equation.jpg" width="30%">
</p>

- **Error Covariance Prediction**:

<p align="center">
  <img src="images/Error_Covariance_Prediction.jpg" width="30%">
</p>

### **5.2. Update Step**

- **Kalman Gain Calculation**:
<p align="center">
  <img src="images/kalman_gain.jpg" width="25%">
</p>

- **State Update**:
<p align="center">
  <img src="images/State_Update.jpg" width="30%">
</p>

- **Error Covariance Update**:

<p align="center">
  <img src="images/Error_Covariance_Update.jpg" width="30%">
</p>

---

## 6. **Examples**

### **6.1. Simple 1D Example**

**Problem**:

Estimate the temperature of a room using noisy thermometer readings.

- **Initial Estimate**: $\hat{T}_0 = 68^\circ \text{C}$
- **Initial Estimate Variance**:$P_0 = 2^\circ \text{C}^2 $
- **Measurement Variance**: $R = 4^\circ \text{C}^2 $
- **Measurements**: $ Y_1 = 75^\circ \text{C}, Y_2 = 71^\circ \text{C}, Y_3 = 70^\circ \text{C} $

**Solution**:

Assume no process noise $( Q = 0 )$ and no control input $( u = 0 )$.

**Time Step 1**:

- **Prediction**:
Prediction_ex_tem.jpg
<p align="center">
  <img src="images/Prediction_ex_tem.jpg" width="20%">
</p>

- **Kalman Gain**:
<p align="center">
  <img src="images/Kalman_Gain_ex_tem.jpg" width="30%">
</p>

- **Update**:
<p align="center">
  <img src="images/Update_ex_tem.jpg" width="70%">
</p>

Repeat these steps for $ Y_2 $ and $ Y_3 $.

|  $k$  | $Y_k$ | $R$ | $\hat{T}_k$ | $P_{k-1}$ | $K$    | $P_k$  |
| :-----: | ------- | ----- | ------------- | ----------- | -------- | -------- |
| $t-1$ |         |       | $68$        | $2$       |          |          |
|  $t$  | $75$  | $4$ | $70.33$     |             | $0.33$ | $1.33$ |
| $t+1$ | $71$  | $4$ | $70.50$     |             | $0.25$ | $1.00$ |
| $t+2$ | $70$  | $4$ | $70.40$     |             | $0.20$ | $0.80$ |
| $t+3$ | $74$  | $4$ | $71$        |             | $0.17$ | $0.66$ |

![single_kf_graph](images/single_kf_graph.png)

### **6.2. 1D Tracking Example**

**Problem**:

Track an object's position and velocity in 1D space with noisy position measurements.
* Given :

<p align="center">
  <img src="images/Given_1d_track.jpg" width="30%"> 
</p>

* Observations:

<p align="center">
  <img src="images/Observations_1d_tra.jpg" width="30%"> 
</p>

* Initial Conditions :

<p align="center">
  <img src="images/Initial_Conditions_1d_tra.jpg" width="30%"> 
</p>

* Process Errors In Process Covariance Matrix

<p align="center">
  <img src="images/Process_Errors_In_Process_Covariance_Matrix_1d_trac.jpg" width="30%"> 
</p>

* Observation Errors :
<p align="center">
  <img src="images/Observation_Errors_1d_trac.jpg" width="30%"> 
</p>

1. The Predicted State

<p align="center">
  <img src="images/The_Predicted_State_1d_trac.jpg" width="40%"> 
</p>

2. The Initial Process Covariance Matrix
   given :

<p align="center">
  <img src="images/Process_Errors_In_Process_Covariance_Matrix_1d_trac.jpg" width="30%"> 
</p>

then

<p align="center">
  <img src="images/Initial_Process_Covariance_Matrix_1d_trac.jpg" width="45%"> 
</p>

3. The Predicted Process Covariance Matrix

<p align="center">
  <img src="images/Predicted_Process_Covariance_Matrix_1d_trac.jpg" width="40%"> 
</p>

4. Calculating the Kalman Gain

<p align="center">
  <img src="images/Kalman_Gain_1d_trac.jpg" width="70%"> 
</p>

5. The New Observation

<p align="center">
  <img src="images/New_Observation_1d_trac.jpg" width="30%"> 
</p>

6. Calculating The Current State
<p align="center">
  <img src="images/The_Current_State_1d_trac.jpg" width="70%"> 
</p>

7. Updating The Process Covariance Matrix
<p align="center">
  <img src="images/update_Process_Covariance_Matrix_1d_trac.jpg" width="60%"> 
</p>
<p align="center">
  <img src="images/Updating_The_Process_Covariance_Matrix_1d_trac2.jpg" width="65%"> 
</p>

* second Round

1. The Predicted State
<p align="center">
  <img src="images/Predicted_State_2nd.jpg" width="50%"> 
</p>

2. The Predicted Process Covariance Matrix
<p align="center">
  <img src="images/Predicted_Process_Covariance_Matrix_2nd.jpg" width="40%"> 
</p>

3. Kalman Gain
 <p align="center">
  <img src="images/Kalman_Gain_2nd.jpg" width="40%"> 
</p>

4. Current Observation
 <p align="center">
  <img src="images/Current_Observation_2nd.jpg" width="40%"> 
</p>

7. Current State Matrix
 <p align="center">
  <img src="images/Current_State_Matrix_2nd.jpg" width="55%"> 
</p>

8. Current Process Covariance Matrix
 <p align="center">
  <img src="images/Current_Process_Covariance_Matrix_2nd.jpg" width="55%"> 
</p>

* third Round
<p align="center">
  <img src="images/predicted_state_3rd.jpg" width="50%"> 
</p>
<p align="center">
  <img src="images/Predicted_Process_Covariance_Matrix_3rd.jpg" width="60%"> 
</p>
<p align="center">
  <img src="images/kalman_gain_3rd.jpg" width="65%"> 
</p>
<p align="center">
  <img src="images/current_observation_3rd.jpg" width="15%"> 
</p>
<p align="center">
  <img src="images/current_state_3rd.jpg" width="55%"> 
</p>
<p align="center">
  <img src="images/current_process_ covariance_3rd.jpg" width="75%"> 
</p>

<!-- ### **6.3. 2D Tracking Example**

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
   $$ -->


**Interpretation**:

The filter estimates both position and velocity, even though only position is measured.

---------------------------------------------------------------------------------------

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
  - *Kalman Filter from the Ground up "* by Alex Becker

- **Tutorials**:

  - *"An Introduction to the Kalman Filter"* by Greg Welch and Gary Bishop.
  - *"THE KALMAN FILTER "* by Michel van Biezen


- **Online Resources**:

  - *"How a Kalman Filter Works, in Pictures"* by Matt B. (For intuitive understanding).
  - *"Sensor Fusion With Kalman Filter"* by Satya


---
