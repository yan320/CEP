# Reinforcement Learning with SAC for Navigaiton


## Paper
Autonomous navigation of mobile robots in unknown environments using off-policy reinforcement learning with curriculum learning


## Libraries

[Pytorch][ROS Noetic][python 3.8]

## ROS 
You can find the packages the I used here:
- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations

```
cd ~/catkin_ws/src/
git clone {link_git}
cd ~/catkin_ws && catkin_make
source ./devel/setup.bash
```

To install my package you will do the same from above.

## Set State

In: turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro.

```
<xacro:arg name="laser_visual" default="false"/>   # Visualization of LDS. If you want to see LDS, set to `true`
```
And
```
<scan>
  <horizontal>
    <samples>360</samples>            # The number of sample. Modify it to 24
    <resolution>1</resolution>
    <min_angle>0.0</min_angle>
    <max_angle>6.28319</max_angle>
  </horizontal>
</scan>
```

## Run Code
I have four stage as in the examples of Robotis.

First to run(cep):
```
roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
In another terminal run:
```
roslaunch project sac_cep_{number_of_stage}.launch
```

First to run(fl):
```
roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
In another terminal run:
```
roslaunch project sac_fl_{number_of_stage}.launch
```

First to run(scf):
```
roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
In another terminal run:
```
roslaunch project scf_stage_{number_of_stage}.launch
```

## If you find it helpful, please cite our work.
@article{YIN2024123202,
title = {Autonomous navigation of mobile robots in unknown environments using off-policy reinforcement learning with curriculum learning},
journal = {Expert Systems with Applications},
volume = {247},
pages = {123202},
year = {2024},
issn = {0957-4174},
doi = {https://doi.org/10.1016/j.eswa.2024.123202},
url = {https://www.sciencedirect.com/science/article/pii/S0957417424000678},
author = {Yan Yin and Zhiyu Chen and Gang Liu and Jiasong Yin and Jianwei Guo}
}
