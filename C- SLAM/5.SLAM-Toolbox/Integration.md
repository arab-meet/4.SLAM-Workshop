# SLAM Toolbox Integration

Author:

Review :

---

**SLAM Toolbox** is a powerful open-source ROS package designed for 2D SLAM. It provides a comprehensive solution for both online and offline SLAM, making it highly versatile for a wide range of applications in autonomous systems, robotics research, and industrial projects. The package combines state-of-the-art algorithms with a user-friendly interface.

The package supports saving and loading maps, allowing users to generate maps once and reuse them for localization purposes, which is invaluable in time-sensitive or resource-constrained applications.

### Why SLAM Toolbox?

SLAM Toolbox stands out for its adaptability and robustness. It can work in complex environments, from tightly confined indoor spaces to expansive and challenging outdoor areas. Its main advantages include:

* **Customizability** : Highly configurable parameters allow users to fine-tune SLAM behavior to meet the needs of specific environments or project requirements.
* **Real-Time Performance** : Efficient algorithms enable online mapping and localization, even in rapidly changing environments.
* **Compatibility with Diverse Sensors** : Compatible with a wide range of sensors, including lidars and depth cameras, to support flexible system integration.

## SLAM Toolbox installation

To install the SLAM toolbox package there are several ways

#### 1. Binary Installation (Recommended)

To install SLAM toolbox using the ROS package manager, use the following command:

```
sudo apt install ros-noetic-slam-toolbox
```

#### 2. Source Installation

Alternatively, you can install SLAM toolbox by cloning the repository directly into your ROS workspace. Follow these steps

**1.Open a terminal and create a new workspace:**

```bash
mkdir -p ~/slam_toolbox_ws/src
cd ~/slam_toolbox_ws
catkin_make
```

**2.Clone the SLAM Toolbox repository from GitHub:**

```bash
cd ~/slam_toolbox_ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git
```

**3.Install any dependencies using `rosdep`:**

```bash
cd ~/slam_toolbox_ws
rosdep install --from-paths src --ignore-src -r -y
```

 4.**Build and source the Workspace**

```bash
catkin_make
source devel/setup.bash
```

### How SLAM Toolbox Works

SLAM Toolbox processes data from sensors (like lidar) and odometry to create and maintain a 2D map while tracking the robot's position within it. Here's a brief breakdown:

1. **Data Collection** : SLAM Toolbox collects sensor and odometry data to detect obstacles and estimate the robot's movement.
2. **Pose Estimation** : It uses scan matching and odometry to estimate the robot's position, aligning each new scan with known map features.
3. **Loop Closure** : When revisiting a known area, SLAM Toolbox detects loop closures to correct mapping drift, improving accuracy.
4. **Pose Graph Optimization** : Detected loop closures trigger optimization algorithms to refine the robot's path and map consistency.
5. **Multi-Session Mapping** : Supports mapping over multiple sessions, saving maps for later use or continuous localization.
6. **Localization Mode** : In localization mode, SLAM Toolbox uses an existing map to track the robot's position without altering the map.

## How to use SLAM Toolbox

SLAM Toolbox provides several tools and modes tailored for different SLAM applications, including online mapping, offline mapping, and localization.

#### **1.Online Mode (Real-Time)**

##### 1.1 Online Async Mode

* **Node** : `online_async.launch`
* **Asynchronous Processing** : In this mode, SLAM Toolbox processes sensor data (like lidar scans) asynchronously, meaning the mapping and pose estimation do not wait for new sensor data to arrive before updating the robot's position and map.
* **Lower Latency** : This is generally faster, with lower latency, making it ideal for real-time mapping when you need quick updates. However, it may introduce minor inconsistencies due to the out-of-sync data.
* **Use Case** : Suitable for real-time mapping in dynamic environments where speed is more critical than precision.
* **Command:**

  ```bash
  roslaunch slam_toolbox online_async.launch
  ```

##### 2.2 Online Sync Mode

* **Node** : `online_sync.launch`
* **Synchronous Processing** : Here, SLAM Toolbox waits for each new scan before updating the map and robot's pose, ensuring each step is aligned with the latest sensor data.
* **Higher Accuracy** : This approach reduces drift and is generally more accurate, as it synchronizes each update with the sensor readings. However, it can be more computationally intensive and might have higher latency.
* **Use Case** : Best for scenarios where accuracy is essential, such as slow-moving robots or environments that require high precision, like indoor mapping.
* **Command:**

  ```bash
  roslaunch slam_toolbox online_sync.launch
  ```

#### 2.Offline Launch File

* **Node** : `offline_sync.launch`
* **Purpose** : This launch file is used for **offline mapping**-processing pre-recorded data (such as **ROS bag files**) to create a map after the data has been collected.
* **How It Works** :
  * The robot will replay the recorded sensor data while building a map from that data.
  * This allows for detailed processing and potentially higher accuracy compared to real-time mapping since you can control the speed of playback and analyze data without real-time constraints.
* **Use Case** : Useful for mapping large environments where data can be collected over time, and the focus is on accuracy rather than real-time processing.
* **Command:**
  ```bash
  roslaunch slam_toolbox offline_sync.launch
  rosbag play <your_bag_file.bag>
  ```

#### 3.Localization Launch File

* **Node** : `localization.launch`
* **Purpose** : This launch file is designed for  **localizing a robot using an existing map** . It assumes that a map has already been created and saved, and the robot will now use this map to determine its position within the environment.
* **How It Works** :
  * The robot subscribes to sensor data and attempts to match it with the existing map.
  * This process allows the robot to calculate its position and orientation based on how the new sensor data aligns with the saved map.
* **Use Case** : Ideal for scenarios where the robot needs to navigate an environment without needing to create a new map, like returning to a known area.
* **Command** :

```bash
roslaunch slam_toolbox localization.launch map_file:=<path_to_saved_map>
```

## Use Instructions

After we installed the SLAM toolbox package and learned the modes and how to use them let's setup our environment and test what we've learned so far.

### 1.Prepare the environment

The first step for using the package is prepare our environment so we should have a robot model and a world to test our package in. Ensure that your robot is equipped with the necessary sensors (like LiDAR or cameras) and that they are properly configured to publish data to the relevant ROS topics (e.g., `/scan` for LiDAR).

For this step we will use our Robot (Rahal Robot) since it has a built in environment that we can use directly.

**1.1 clone rahal repository and build your workspace:**

```bash
cd ~/slam_toolbox_ws/src
git clone https://github.com/arab-meet/Rahal_Robot
cd ..
catkin_make
source devel/setup.bash
```

For further instruction regarding the robot you can check the repository [here](https://github.com/arab-meet/Rahal_Robot)

**1.2 launch gazebo world:**

```bash
roslaunch rahal_description_pkg rahal_gazebo_world.launch
```

### 2.configure the launch file

### 3.Creating the map

### 4.Saving the map
