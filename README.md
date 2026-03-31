# erc_mars_rover_navigation

This is the ROS 2 repository for creating a self-driving mars rover.

## Setup

This ROS 2 project is set to run on an NVidia Jetson Nano Orin Super with NVidia flavoured Ubuntu 22.04 LTS. The lidar sensor used is a Unitree 4D Lidar L2.

## High Level Navigation Algorithm

(Rover placed in starting location)

1. Mark starting location as the origin
2. Internally set the first waypoint / aruco marker to detect
3. Auto Navigate for a certain amount of time
4. Raise camera arm and spin camera to search for the current waypoint goal
5. If waypoint found, goto 6. else go to 3. 
6. Create RRT* path from current position to end position
7. Traverse RRT* path and dock in front of waypoint
8. Mark current waypoint as found
9. If all the waypoints are found, goto 10., else go 4.
10. Create RRT* path from current position to origin and traverse path
11. Stop