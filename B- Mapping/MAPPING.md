# Mapping: 

Author: mo3taz , Eng Wafaa

## Understanding the Mapping Problem

Imagine you are a **pirate** in search of **treasure**. What would you need to guide you to your ultimate prize?

![Pirate](images/pirate.jpg)
![Treasure](images/treasure.jpg)

The answer is simple: a **map**.

This is precisely the same problem faced by mobile robots. They need a map to know where they are (localization) and where they need to go.

![Map](images/map.jpg)

## Mapping in Mobile Robotics

Mapping in mobile robotics refers to the process of creating a digital representation of the environment in which a robot operates. It's a crucial component of robot navigation and interaction with the world.

### Robots Without a Map

![Where am I?](images/where_am_i.gif)

Without a map, a robot is lost, just like a pirate without a treasure map. We need to provide the robot with a map to help it understand its surroundings and navigate effectively.

The robot's "brain" is a computer, and computers can't inherently comprehend the environment as it is. We need to convert the environment into a form that the computer can understand, like a grid of 0s and 1s. This digital representation is called a map.

## Types of Mapping

There are several types of mapping techniques used in mobile robotics:

1. **Topological Mapping**:
   Represents the environment as a graph of connected locations (stations).
   ![Topological Map](images/topological_representation.png)

   The scale in a topological map doesn't need to be accurate, making it a lightweight representation. However, this representation doesn't provide enough detail to navigate from one location to another, which is where a **metric representation** becomes useful.

2. **Metric Mapping**:
   This representation uses precise coordinates, like longitude and latitude, to map the environment. While it's more accurate, it's also more sensitive to noise.
   ![Metric Map](images/world.png)

## Occupancy Grid Maps

Occupancy grid maps are a popular type of metric map used in mobile robotics. They divide the environment into a grid of cells, where each cell represents the probability of that space being occupied by an obstacle.

![Occupancy Grid Map](images/OGM.png)

Occupancy grid mapping implements the binary Bayes filter to estimate the occupancy value of each cell.

Each cell in the grid has an occupancy variable:
- `mₓ,y = 1`: The cell is believed to be occupied (there is an obstacle).
- `mₓ,y = 0`: The cell is believed to be free (there is no obstacle).

![Occupancy Grid Map](images/map.png)

## Measurement Model (`p(z|mₓ,y)`)

The measurement model describes the probability of getting a sensor measurement `z` given the occupancy state `mₓ,y` of a grid cell.

![Bayes' Rule](images/Bayes_Rule.png)

The key probabilities in the measurement model are:

1. **True Occupied Measurement**: `p(z = 1 | mₓ,y = 1)`
   - This is the probability that the sensor measurement `z` detects an obstacle (`z = 1`) given that the grid cell at `(x, y)` is actually occupied (`mₓ,y = 1`).
   - This probability is typically high, meaning the sensor is likely to correctly detect an obstacle when the grid cell is truly occupied.

2. **False Free Measurement**: `p(z = 0 | mₓ,y = 1)`
   - This is the probability that the sensor measurement `z` indicates the space is free (`z = 0`) even though the grid cell at `(x, y)` is actually occupied (`mₓ,y = 1`).
   - This probability is typically low, as it's an error case where the sensor fails to detect an obstacle in an occupied cell.

3. **False Occupied Measurement**: `p(z = 1 | mₓ,y = 0)`
   - This is the probability that the sensor measurement `z` indicates the space is occupied (`z = 1`) even though the grid cell at `(x, y)` is actually free (`mₓ,y = 0`).
   - This probability is typically low, as it's an error case where the sensor falsely detects an obstacle in a free space.

4. **True Free Measurement**: `p(z = 0 | mₓ,y = 0)`
   - This is the probability that the sensor measurement `z` indicates the space is free (`z = 0`) given that the grid cell at `(x, y)` is actually free (`mₓ,y = 0`).
   - This probability is typically high, as the sensor correctly identifies that the space is unoccupied.

By understanding these measurement probabilities, we can use the Bayes filter to update the occupancy estimates in the grid map as the robot moves and collects new sensor data.

### Mapping in real world

![Mapping GIF](images/mapping2.gif)

# [Next Topic Link]

## References:

- [Back to main](../README.md)