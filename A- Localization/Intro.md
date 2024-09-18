# Localization

*Localization theory for mobile robots involves algorithms and mathematical models used to estimate the robot's position and orientation within an environment. It integrates data from various sensors—like GPS, LIDAR, and odometry—to provide accurate positioning. Key concepts include probabilistic methods (for example the most two common algorithms are Monte Carlo Localization and Extended Kalman Filter), which account for uncertainties and noise in sensor measurements, and filtering techniques ( for example Kalman Filters) that combine sensor data with a robot's motion model to refine position estimates. This theory helps robots navigate complex environments by continuously updating and correcting their position in real-time.*

![alt text](images/Loca2.jpg)


### There are three different types of localization chalanges:

**1-Local Localization:** *often called position tracking, involves assessing a robot's pose as it moves through its environment. This process relies on the robot's awareness of its **initial pose**,filters such as the **Kalman Filter** can be employed to minimize inaccuracies in sensor readings, addressing the challenges associated with sensor noise.*

![alt text](images/Glob&Loca.jpg)

**2-Global Localization:** *the robot starts with an unknown initial pose and must determine its position relative to a ground truth map. to addressing this challenge, algorithms such as the **Extended Kalman Filter, Monte Carlo methods,** and others can be utilized.*

**3-Kidnapped Robot:** *this problem resembles global localization, but with the added complexity that the robot can be unexpectedly "kidnapped" and relocated to a different spot on the map at any time, then correctly locate itself again on a map.* 


**Note:** In this course, we will focus exclusively on **static** (unchanging) environments, where conditions consistently align with the ground truth map. In contrast, **dynamic** environments involve objects within the map that may move over time, making localization and navigation more complex.


## Sensors Used in Localization:

#### 1-Laser Scanners (LIDAR)

![alt text](images/Lidar.jpg)

* Description: Uses laser beams to scan the environment and create a detailed 2D or 3D map.

* Function: Provides precise distance measurements to obstacles, which can be used to compare with a known map to estimate the robot’s position.

* Advantages: High accuracy and can detect obstacles even in cluttered environments.


#### 2-Cameras (Visual Odometry) 

![alt text](images/Camera.jpg)

* Description: Uses visual data from cameras to estimate movement and orientation.

* Function: Analyzes changes in the visual input to infer the robot's movement relative to its starting point.

* Advantages: Can provide rich environmental details and work in diverse lighting conditions.


#### 3-IMUs (Inertial Measurement Units)

![alt text](images/Imu.jpg)

* Description: Measures acceleration and angular velocity.

* Function: Helps in tracking changes in orientation and movement, complementing odometry data to improve localization accuracy.

* Advantages: Provides additional information about the robot’s movement that can be useful in dynamic environments.

#### 4- Wheel Encoders:

![alt text](images/Encoder.jpg)

* Description: These encoders are attached to the robot's wheels or axles. They track the number of wheel rotations or partial rotations by generating pulses as the wheels turn.

* Function: By counting the pulses, the robot can calculate how far it has traveled. Combining this data with the wheel's diameter, the robot estimates its linear displacement.

* Advantages: Provides straightforward and effective measurements of distance traveled. Useful for odometry calculations.

* Limitations: Susceptible to errors from wheel slippage, uneven terrain, or wear and tear.

#### 5-GPS (Global Positioning System)

![alt text](images/GPS.jpg)

* Description: Uses satellites to provide location data based on geographical coordinates.

* Function: Offers global positioning with high accuracy outdoors, helping robots determine their position on a global scale.

* Limitations: Less effective indoors or in areas with poor satellite visibility.