
# Mapping

Author: mo3taz , Eng Wafaa

## Understanding the Mapping Problem

### Imagine you are a pirate

![Pirate](./images/pirate.jpg)

### And your goal is to find treasure

![Treasure](./images/treasure.jpg)

### What would you need to get to the treasure?

The answer is a **map**.

![Map](./images/map.jpg)

This is the same problem that a robot faces. It needs a map to know where it is (localization) and where it will go.

### Robots Without a Map

![Where am I?](./images/where_am_i.gif)

Because of this, we need a map of our environment to allow robots to know their location on this map.

The robot's "brain" is a computer, and the computer can't understand the environment as it is. We need to convert this environment into something that the computer can understand, like `0` or `1`. This is a map - a digital representation of the environment.

## Types of Mapping

There are several types of mapping techniques used in mobile robotics:

1. **Topological Mapping**: Represents the environment as a graph of connected locations (stations).

   ![Topological Representation](./images/topological_representation.png)

   The scale here doesn't need to be accurate, so this representation is lightweight. However, as you can see, there is not much detail to allow the robot to go from the green station to the red station. Because of this, a more useful representation is **Metric Mapping**.

2. **Metric Mapping**: This representation uses precise coordinates, like longitude and latitude coordinates. But because this representation is precise, it's pretty sensitive to noise.

   ![Metric Mapping](./images/world.png)

3. **Feature-Based mapping:** This mapping technique focuses on specific features or landmarks in the environment. These features are stored as points, lines, or keypoints that we use to define our environment.

<img src="./images/feature_based_mapping.png" />

4. **Occupancy Grid Mapping:** This divides the environment into a grid of cells, with each cell representing whether it's occupied, free, or unknown.

<img src="./images/occupancy_grid_mapping.png" />

> **There's many types of mapping each with it's pros, cons and use cases. we will focus on the most common mapping approach which is occupancy grid mapping.**

## Occupancy Grid Maps

Occupancy grid maps are a popular type of metric map used in mobile robotics. They divide the environment into a grid of cells, where each cell represents the probability of that space being occupied by an obstacle.

Occupancy grid mapping is an approach to create a map. It implements the binary Bayes filter to estimate the occupancy value of each cell.

![Occupancy Grid Map](./images/OGM.png)

So, it's working as follows: each cell will have an occupancy variable:

![Occupancy Map](./images/map.png)

- **m(x,y) = {free, occupied} -> {0, 1}**
  - **m(x,y)**: This is the state of the grid cell at position (x, y). It can take two values:
    - **Black Color** `m(x,y) = 1`: The grid cell is believed to be occupied (there is an obstacle).
    - **White Color** `m(x,y) = 0`: The grid cell is believed to be free (there is no obstacle).

![Occupancy Grid Map](./images/ogm_map.png)

### Probability

![Bayes' Rule](./images/Bayes_Rule.png)

**Posterior: p(m(x,y)|z)**
- Probability that a cell (x,y) is occupied given the sensor measurement z
- This is what we want to calculate for each cell in the grid map

**Likelihood: p(z|m(x,y))**
- Probability of getting the sensor measurement z if the cell (x,y) is occupied
- Models the sensor's behavior and accuracy

**Prior: p(m(x,y))**
- Initial belief about the occupancy of cell (x,y) before considering the sensor data
- For example, 0.5 for occupied and 0.5 for free

**Evidence: p(z)**
- Probability of getting the sensor measurement z regardless of the cell's state

### Measurement Model p(z|m(x,y))

In an occupancy grid map, we are trying to estimate whether a grid cell is **occupied** or **free** based on sensor measurements. Here's what each term means:

- **z**: This is the sensor measurement (the reading from a sensor like LiDAR, sonar, or depth camera). It can take two values:
  - `z = 1`: The sensor detects that the space is occupied (e.g., the sensor has "hit" an obstacle).
  - `z = 0`: The sensor detects that the space is free (e.g., the sensor has not detected an obstacle).

#### True Occupied Measurement: p(z = 1 | m(x,y) = 1)

This is the probability that the sensor measurement **z** detects an obstacle (`z = 1`) given that the grid cell at `(x, y)` is actually occupied (`m(x,y) = 1`).

- **True occupied measurement**: This happens when both the sensor reading and the actual state of the grid cell indicate occupancy (i.e., `z = 1` and `m(x,y) = 1`).
  - This probability is typically high, meaning if the grid cell is truly occupied, the sensor is very likely to detect it as occupied.

1. **p(z = 1 | m(x,y) = 1)**
   - This is a **True occupied** measurement: It represents the probability that the sensor measurement **z** indicates the space is occupied (`z = 1`) given that the grid cell at position `(x, y)` is actually occupied (`m(x,y) = 1`).
   - This probability is typically high, meaning the sensor is likely to correctly detect an obstacle when the grid cell is truly occupied.

2. **p(z = 0 | m(x,y) = 1)**
   - This is a **False free** measurement: It represents the probability that the sensor measurement **z** indicates the space is free (`z = 0`) even though the grid cell at position `(x, y)` is actually occupied (`m(x,y) = 1`).
   - This probability is typically low, as it's an error case where the sensor fails to detect an obstacle in an occupied cell.

3. **p(z = 1 | m(x,y) = 0)**
   - This is a **False occupied** measurement: It represents the probability that the sensor measurement **z** indicates the space is occupied (`z = 1`) even though the grid cell at position `(x, y)` is actually free (`m(x,y) = 0`).
   - This probability is typically low, as it's an error case where the sensor falsely detects an obstacle in a free space.

4. **p(z = 0 | m(x,y) = 0)**
   - This is a **True free** measurement: It represents the probability that the sensor measurement **z** indicates the space is free (`z = 0`) given that the grid cell at position `(x, y)` is actually free (`m(x,y) = 0`).
   - This probability is typically high, as the sensor correctly identifies that the space is unoccupied.

### Mapping look like: 
![mapping 2](images/mapping2.gif)

## Limitation

One noticable limitation for occupancy grid maps which is computational power specially in large environment and if we want to get a high resolution map.

### Computational power v.s. accuracy

let's assume that we have a robot that has a start point and goal point which the robot can reach and we will devide our map into relativly small grids to get a good enough accuracy.

![topo map](./images/small_grid.png)

In the previous image we notice that the robot can reach the goal point after the mapping process but we devided our map to 72 cells which consumes a lot of computational resources.

---

Let's double the size of our grids now we are using just 20 grids for the whole map. we greatly reduced the computational power but we got low accuracy causing our obstacle to block the robot path.

![topo map](./images/big_grid.png)

---

One approach to solve this problem is using grids with different sizes notice here we started with 20 grids to reduce the computational power as much as possible but we know that it won't give us the accuracy that we desire so we devide the occupied grids only into smaller cells since it doesn't have a meaning to devide the empty cells. and we recalculate the values of the divided cells and so on until we reach the accuracy that we need.

we notice here that we reached the same accuracy as the first image with only 44 cells instead of 72. So we reached a good accuracy with low computational power.

![topo map](./images/dynamic_size_grid.png)

# References:

### [&lt;-Back to main](../README.md)

