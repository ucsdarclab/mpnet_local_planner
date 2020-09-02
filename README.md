# DynamicMPNet

This repository aims to document the setup and usage of the Dynamic MPNet Planner for the purpose of robot navigation.

## Dependencies

The planner was tested on a Docker Container with the following specifications.

1. ROS-Melodic
2. LibTorch Build Version 1.5.1
3. CUDA Version 10.2
4. [mit-racecar simulator](https://github.com/jacobjj/racecar_simulator/tree/navigation)
   
## Package Installation
Git clone the package your catkin workspace.

```
cd ~/catkin_ws/src
git clone https://github.com/jacobjj/mpnet_local_planner.git
```
More instructions to come.

## Running the simulation

```
roslaunch mpnet_plan simulate_mpnet.launch
```

This should spawn off an instance of RViz, where you can set the 2D Nav Goal and see the mpnet planner at work!

   
   
   

