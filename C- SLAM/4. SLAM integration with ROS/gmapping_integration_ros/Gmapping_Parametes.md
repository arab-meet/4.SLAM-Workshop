## GMapping Launch File Explaination

**use_sim_time** : This parameter is set to `true` to indicate the use of simulation time instead of real time if you are using it in real set it to false.

**slam_gmapping node:** Launches the GMapping node responsible for SLAM.

```xml
<remap from="scan" to="/scan"/>
```

 Specifies the input laser scan topic name which is expected to be  `/scan`".

### Parameters Explaination

#### `base_frame` (string, default: "base_link")

* **Purpose:** Specifies the name of the coordinate frame attached to the mobile base of the robot.
* This frame represents the base of the robot and is important for mapping and localization.

#### `map_frame` (string, default: "map")

* **Purpose:** Specifies the name of the coordinate frame attached to the map.
* This frame represents the global map frame, which is used to align all sensor data within the map's coordinate system.

#### `odom_frame` (string, default: "odom")

* **Purpose:** Specifies the name of the coordinate frame attached to the odometry system.
* This frame represents the odometry reference frame, providing continuous position and orientation updates of the robot.

#### `map_update_interval` (float, default: 5.0)

* **Purpose:** Sets the interval (in seconds) between updates to the occupancy grid map.
* Lowering this value increases map update frequency but also increases computational load, impacting real-time performance.

#### `maxUrange` (float, default: 80.0)

* **Purpose:** Specifies the maximum usable range of the laser sensor.
* Laser beams beyond this range are cropped from the scan data to improve mapping accuracy and reduce noise.

#### `sigma` (float, default: 0.05)

* **Purpose:** Sets the sigma (standard deviation) used for the greedy endpoint matching during scan matching.
* This parameter affects how well the algorithm matches observed laser points to predicted points.

#### `kernelSize` (int, default: 1)

* **Purpose:** Specifies the size of the kernel used to look for correspondence during scan matching.
* Increasing this parameter can help in matching laser scans more accurately but may increase computational cost.

#### `lstep` (float, default: 0.05)

* **Purpose:** Sets the optimization step size in translation during scan matching.
* This parameter affects the rate at which the algorithm adjusts the robot's estimated position.

#### `astep` (float, default: 0.05)

* **Purpose:** Sets the optimization step size in rotation during scan matching.
* This parameter affects the rate at which the algorithm adjusts the robot's estimated orientation.

#### `iterations` (int, default: 5)

* **Purpose:** Specifies the number of iterations used for refining the scan matching process.
* Increasing iterations can improve the accuracy of pose estimation but may increase computational time.

#### `lsigma` (float, default: 0.075)

* **Purpose:** Sets the sigma (standard deviation) of a laser beam used for likelihood computation.
* This parameter influences how sensor noise is considered during the likelihood computation.

#### `ogain` (float, default: 3.0)

* **Purpose:** Sets the gain used for evaluating the likelihood to smooth resampling effects.
* Increasing this parameter can amplify the influence of scan data on map updates.

#### `lskip` (int, default: 0)

* **Purpose:** Specifies the number of laser beams to skip during likelihood computation.
* Adjusting this parameter can reduce computational load by processing fewer laser beams.

#### `minimumScore` (float, default: 0.0)

* **Purpose:** Sets the minimum score threshold for accepting scan matching results.
* **Details:** Higher values can filter out unreliable scan matches, improving map accuracy in open areas.

#### `srr` (float, default: 0.1)

* **Purpose:** This parameter represents the odometry error in translation relative to translation (**rho**/**rho**).
* It quantifies how the translational error scales with the magnitude of the translation (**rho**). A higher value indicates increased translation error with larger movements.

> "rho" represents the linear distance traveled by the robot along its current heading or direction.

#### `srt` (float, default: 0.2)

* **Purpose:** This parameter signifies the odometry error in translation relative to rotation (**rho**/**θ**).
* It specifies how the translational error scales with the rotational angle (**θ**). A higher value implies more significant translation error for rotations.

#### `str` (float, default: 0.1)

* **Purpose:** This parameter denotes the odometry error in rotation relative to translation (**θ**/**rho**).
* It describes how the rotational error scales with the magnitude of translation (**rho**). A larger value indicates increased rotational error for longer movements.

#### `stt` (float, default: 0.2)

* **Purpose:** This parameter indicates the odometry error in rotation relative to rotation (**θ**/**θ**).
* It specifies how the rotational error scales with the rotational angle (**θ**). A higher value suggests more significant rotational error for larger rotation angles

#### `linearUpdate` (float, default: 1.0)

* **Purpose:** Sets the linear update threshold for processing scans based on robot translation.
* This parameter determines how far the robot must translate before processing a new scan.

#### `angularUpdate` (float, default: 0.5)

* **Purpose:** Sets the angular update threshold for processing scans based on robot rotation.
* This parameter determines how much the robot must rotate before processing a new scan.

#### `temporalUpdate` (float, default: -1.0)

* **Purpose:** Sets the temporal update threshold for processing scans based on time.
* A negative value disables time-based updates, while a positive value triggers updates after a specified time interval.

#### `resampleThreshold` (float, default: 0.5)

* **Purpose:** Sets the Neff-based resampling threshold for particle filtering.
* Higher values trigger particle resampling less frequently, affecting the robustness of the particle filter.

#### `particles` (int, default: 30)

* **Purpose:** Specifies the number of particles used in the particle filter for pose estimation.
* Increasing the number of particles can improve pose estimation accuracy but requires more computational resources.

#### `Xmin`:

* Defines the minimum x-coordinate boundary of the map.

#### `ymin`:

* Defines the minimum y-coordinate boundary of the map.

#### `xmax`:

* Defines the maximum x-coordinate boundary of the map.

#### `ymax`:

* Defines the maximum y-coordinate boundary of the map.
