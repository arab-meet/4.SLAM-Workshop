# GraphSLAM(Cartographer)

Author: zaynap, Abdelrahman Eldesouky

## Introduction

GraphSLAM is an algorithm designed to solve the full SLAM problem, similar to FastSLAM. It recovers  **all robot poses and the entire map** , unlike online SLAM, which only recovers the **most recent pose and map**.

![1725907302796](image/GraphSLAM(Cartographer)/1725907302796.png)

GraphSLAM relies on dependencies between current and previous poses, improving upon the accuracy of FastSLAM. In large environments, FastSLAM may struggle because it uses a finite number of particles, and at certain points, there may not be a particle at the most likely location. FastSLAM works with bits of information, but GraphSLAM significantly improves this by using a more robust method for handling pose and map estimation.

![1725907573229](image/GraphSLAM(Cartographer)/1725907573229.png)

## GraphSLAM and Graphs

As the name suggests, GraphSLAM uses **graphs** to represent the robot’s poses and its environment. Each pose or feature is represented as  **nodes** , and the connections between them are represented by  **edges (constraints)** :

* **Motion constraints** : Represented by a solid line between two robot poses.
* **Measurement constraints** : Represented by a dashed line between a robot pose and an environmental feature.

![1725907884796](image/GraphSLAM(Cartographer)/1725907884796.png)

As the robot moves, new nodes are added to the graph, expanding it as more information is gathered.

![1725907954696](image/GraphSLAM(Cartographer)/1725907954696.png)

## Front-End vs. Back-End in GraphSLAM

The goal of GraphSLAM is to construct a **graph** that represents all the robot's poses and environmental features, then use this graph to find the **most likely robot path** and **accurate map** of the environment. To achieve this, GraphSLAM is divided into two major components: the **front-end** and the  **back-end**.

![1725908370539](image/GraphSLAM(Cartographer)/1725908370539.png)

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

### Back-End

The **back-end** of GraphSLAM takes the complete graph from the front-end, which includes all the robot poses and environmental features connected by constraints, and performs  **graph optimization** . The goal of the back-end is to find the most likely configuration of robot poses and environmental features, minimizing the error in the graph.

This optimization process involves:

* **Minimizing constraint errors** to produce a consistent map and trajectory.
* Solving the graph using techniques like **Maximum Likelihood Estimation (MLE)** to ensure the solution is as accurate as possible.

Graph optimization is central to the back-end, as it refines the entire graph to ensure consistency across all nodes and edges, ultimately leading to the most probable estimate of the robot’s path and the map of its environment.

## GraphSLAM Workflow

The front-end and back-end of GraphSLAM can be completed in two ways:

1. **In order (sequentially)** .
2. **Iteratively** , refining the results over time.

## Graph Optimization

The core of GraphSLAM lies in  **graph optimization** , ensuring that the errors in constraints are minimized to provide the most accurate representation of the robot's path and the environment. By applying optimization techniques such as MLE, GraphSLAM offers a robust and scalable solution to the full SLAM problem, particularly in large-scale environments where traditional particle-based approaches like FastSLAM may struggle.

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
