# Mapping 

Author:MO3TAZ

Review :

## Understand Mapping problem?

### imaging with me you are a **pirate**

<img src="./images/pirate.jpg" alt="pirate" width="200"/>

### and your goal is **treasure**

<img src="./images/treasure.jpg" alt="treasure" width="200"/>

### What would you need to get to the treasure?

**answer is map**

<img src="./images/map.jpg" alt="map" width="200"/>


**this is same problem that robot face**
**He need a map to able to know where he is `(localization)` and where he will go**


Mapping in mobile robotics refers to the process of creating  representation of the environment in which a robot operates. It's a crucial component of robot navigation and interaction with the world.

### Robots without Map:

![where am i](./images/where_am_i.gif)

becouse of that we need Map for our enviroment our world to allow robots knows his location on this map.

brain of the robot is computer and the computer can't understand the environment as it is, we need to convert this environment to something that computer can understand like `0 or 1`, this is a map it is `digital representation` for environment.

**image for map**


## Types of Mapping

There are several types of mapping techniques used in mobile robotics:
1. **Topological Mapping**: Represents the environment as a graph of connected locations(stations).
![topo map](./images/topological_representation.png)

scale here doesn't need to be accurate becouse of that this representation is lightweight
but as you can see there is no much details to allow you to go to from green station to red station 
becouse of that there is useful representation is **metric representation**

2. **Metric Mapping**: this representation use precise coordinates like longitude and latitude coordinates but becouse this representation is precise it's pretty sensitive to noise 
![topo map](./images/world.png)



## Occupancy Grid Maps

Occupancy grid maps are a popular type of metric map used in mobile robotics. They divide the environment into a grid of cells, where each cell represents the probability of that space being occupied by an obstacle.


- occupancy grid mapping is an approach to create map it implements the binary bayes filter to estimate the occupancy value of each cell.

<img src="images/OGM.png" alt="drawing" width="200" height="200"/>

**So it's working as following each cell will have occupancy variable :**

<img src="images/map.png" alt="drawing"/>

**m(x,y) = {free , occupied} -> {0,1}**

- **mₓ,y**: This is the state of the grid cell at position (x, y). It can also take two values:
  -**Black Color** `mₓ,y = 1`: The grid cell is believed to be occupied (there is an obstacle).
  -**White Color** `mₓ,y = 0`: The grid cell is believed to be free (there is no obstacle).

<img src="images/ogm_map.png" alt="drawing"/>

#### Probability

![drawing](images/Bayes_Rule.png)

## Posterior: p(m_x,y|z)
- Probability that a cell (x,y) is occupied given the sensor measurement z
- This is what we want to calculate for each cell in the grid map

## Likelihood: p(z|m_x,y)
- Probability of getting the sensor measurement z if the cell (x,y) is occupied
- Models the sensor's behavior and accuracy

## Prior: p(m_x,y)
- Initial belief about the occupancy of cell (x,y) before considering the sensor data
- for example  0.5 for occupied and 0.5 Free
## Evidence: p(z)
- Probability of getting the sensor measurement z regardless of the cell's state


#### Measurement Model p(z|m_xy)

In an occupancy grid map, we are trying to estimate whether a grid cell is **occupied** or **free** based on sensor measurements. Here's what each term means:

- **z**: This is the sensor measurement (the reading from a sensor like a LiDAR, sonar, or depth camera). It can take two values:
  - `z = 1`: The sensor detects that the space is occupied (e.g., the sensor has "hit" an obstacle).
  - `z = 0`: The sensor detects that the space is free (e.g., the sensor has not detected an obstacle).


## True Occupied Measurement: p(z = 1 | mₓ,y = 1)

This is the probability that the sensor measurement **z** detects an obstacle (`z = 1`) given that the grid cell at `(x, y)` is actually occupied (`mₓ,y = 1`).

- **True occupied measurement**: This happens when both the sensor reading and the actual state of the grid cell indicate occupancy (i.e., `z = 1` and `mₓ,y = 1`).
  - This probability is typically high, meaning if the grid cell is truly occupied, the sensor is very likely to detect it as occupied.



1. **p(z = 1 | mₓ,y = 1)**

   * This is a **True occupied** measurement: It represents the probability that the sensor measurement **z** indicates the space is occupied (z = 1) given that the grid cell at position (x, y) is actually occupied (mₓ,y = 1).

   * This probability is typically high, meaning the sensor is likely to correctly detect an obstacle when the grid cell is truly occupied.

2. **p(z = 0 | mₓ,y = 1)**

   * This is a **False free** measurement: It represents the probability that the sensor measurement **z** indicates the space is free (z = 0) even though the grid cell at position (x, y) is actually occupied (mₓ,y = 1).

   * This probability is typically low, as it's an error case where the sensor fails to detect an obstacle in an occupied cell.

3. **p(z = 1 | mₓ,y = 0)**

   * This is a **False occupied** measurement: It represents the probability that the sensor measurement **z** indicates the space is occupied (z = 1) even though the grid cell at position (x, y) is actually free (mₓ,y = 0).

   * This probability is typically low, as it's an error case where the sensor falsely detects an obstacle in a free space.

4. **p(z = 0 | mₓ,y = 0)**

   * This is a **True free** measurement: It represents the probability that the sensor measurement **z** indicates the space is free (z = 0) given that the grid cell at position (x, y) is actually free (mₓ,y = 0).
   * This probability is typically high, as the sensor correctly identifies that the space is unoccupied.


**gif for creating map for rahal robot**




# [Next Topic Link]

# References:

### [&lt;-Back to main](../README.md)