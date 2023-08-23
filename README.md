# Straberry-Stacker
Self Project For simulation of drone in Gazebo
# Theme description
Commercial strawberry harvesting necessitates expert labour. The picker must swiftly identify any ripe strawberries on a plant, select them, and pack the berries into boxes all at the same time. This is labour for the employees as well as bad for the farm's overall efficiency.

The aim is to build a multi-drone system for picking strawberry boxes from a field and stacking them onto a transport trailer. A total of 6 incremental tasks starting from the installation to the final solution to the problem are detailed here.

# Software Descriptions which are used in this project
We will use Gazebo simulator, a robotics simulator, where the simulated farm and UAVs will dwell, the PX4 Autopilot ecosystem for controlling the UAV, and ROS for integrating the many parts of autonomy required in the solution.

# 1. Ubuntu 20.04 LTS
Ubuntu 20.04, a Linux environment is used for running all the packages and programmes such as Gazebo 11 etc.
# 2. ROS Noetic
The Robot Operating System (ROS) is a set of software libraries and tools that help one build robot applications. Note: ROS is strongly version specific middleware. Thus, Ubuntu 20.04 (Focal) is used with ROS Noetic.
# 3. Gazebo
Gazebo 11, a physical engine (used for simulation) is tightly integrated with ROS Noetic and so it comes pre-installed when ros-noetic-desktop-full is installed.
# 4. PX4 Autopilot
PX4 is an open source flight control software for drones and other unmanned vehicles.
It provides a standard to deliver drone hardware support and software stack, allowing an ecosystem to build and maintain hardware and software in a scalable way.
# 5. QGroundControl
QGroundControl provides full flight control and mission planning for any MAVLink enabled drone.
It acts as a Ground Control Station (GCS) of the drone.
#6. Python3
All the programs interfacing with ROS Noetic framework are written in Python3.
It comes preinstalled with ROS Noetic.
