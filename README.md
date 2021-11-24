# imagine_ros

Robots simulate their sensors in triangle meshes.

ROS integration of **imagine** library and examples for:
- Basic usage of imagine library
- Simulating different attributes for intersections
- Advanced examples to speed up your applications 

## Dependency
imagine library to be installed

## Nodes

TODO

### lidar_simulator
Simulate a lidar sensor. 

### depth_camera_simulator
Simulate a depth camera sensor.

### camera_simulator
Simulate a camera sensor.

## Possible applications

### Reinforcement Learning (RL) on real robots

Choosing an action depending on your sensory data. Given a list of actions we change our state uncertainly. Possible sensory data has to therefore simulate from a lot of possible states to choose the best next action. With **imagine** the simulation time becomes so low that even on a mobile robot such methods can be applied (See perfomance section of imagine-library).

### Probabilistic Localization

AMCL in triangular meshes. See mamcl. 




