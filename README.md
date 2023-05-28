# Autonomous Parking Using Hybrid A* Search

## Problem Statement
A common path planning problem for autonomous vehicles involves maneuvering in tight spaces and cluttered environments, particularly while parking. To address these issues, non-holonomic constraints of the vehicles must be taken into consideration. At the same time, collision checking algorithm must also be deployed to make sure the vehicle path is collision-free. Considering the above constraints, this project implements path planning for the following types of vehicles:
* An autonomous delivery robot with DiWheel Kinematics
* An autonomous car with Akerman Steering
* An autonomous truck pulling a trailer

## About the environment
The project is implemented using the pygame library of python. The environment for the same is a parking lot in a two dimensional grid world. In each instance, a car starts at the Northwest corner of the bounded two dimensional field. The car is to be parked in a compact space in the southern border, flanked by vehicles both in front of and behind the target space. Additionally, there is an island in the central region. The environment approximately looks like the following:

![Environment](https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/27047065-5cca-4541-8e96-4c971ec65584)


## An autonomous delivery robot with DiWheel Kinematics
  This delivery robot is a robot having two parallel wheel side by side. Robots having diwheel kinematics have the capability of taking spot turns, unlike autonomous cars. A figure showing diwheel kinematics is displayed below:

![Diwheel](https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/ab6b4a5f-c055-4256-a27e-6d66a217c20e)


Here is an example of the simulation for an autonomous delivery robot with Diwheel kinematics:



https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/f9a3052e-ad2e-43ed-8f22-8813c29e36f2



## An autonomous car with Akerman Steering

Akerman Steering mechanism is used to some extent in today's cars. The following figure depicts akerman steering:

![Akerman](https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/c2131664-e67a-4b98-853d-da6ff63a0a35)


Here is an example of the simulation for an autonomous car with Akerman steering:



https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/49e994f4-b022-49b7-a868-734243d7c613



## An autonomous truck pulling a trailer

This truck has a trailer attached at the back of the iy. The following figure depicts an autonomous truck pulling a trailer:

![Trailer](https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/b36e4689-20e7-47b3-964b-92a2f5347c02)



Here is an example of the simulation for an autonomous truck pulling a trailer:



https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/assets/134632027/42159d81-1204-48fc-b2c0-317797cde3d0


## Report

Please find the more detail on path depiction and algorithm's performance in the [report.](https://github.com/kt-krutarthtrivedi/Autonomous-Parking-Using-A-Star/blob/main/media/Report.pdf)




## References
* [Steven M. LaValle. Planning Algorithms. Cambridge University Press, May 2006. ISBN 9780521862059.](http://lavalle.pl/planning/)

