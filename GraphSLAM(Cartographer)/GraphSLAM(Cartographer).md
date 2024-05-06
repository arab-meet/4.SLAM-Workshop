# GraphSLAM(Cartographer)

Author: zaynap 

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

![1715037001254](image/GraphSLAM(Cartographer)/1715037001254.png)i

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

### [&lt;-Back to main](../README.md)
