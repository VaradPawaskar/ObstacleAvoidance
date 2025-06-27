# Obstacle Avoidance Robot Simulation  

_Building a differential drive robot which can avoid obstacles. The robot design and other requirements can be found in this [PDF](https://github.com/saharshleo/obstacleAvoidanceRobot/blob/master/ProblemStatement.pdf)._  
This work was done taking reference from [here](https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1).

## Prerequisites  

* [ROS](http://wiki.ros.org/kinetic)  
* [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs)

## Getting Started

1. Clone this repository.
2. Run `catkin_make` for both `catkin_ws` and `simulation_ws`.
3. Launch your terminal and run the command `roslaunch my_worlds <world_name>.launch`. 
This will launch the gazebo enviroment
4. In another terminal, run the command `roslaunch robot_description spawn.launch`. 
This will load the robot in the environment at origin. It can be spawned at different location by giving additional arguments like `x:=3 y:=8 z:=7`.  
5. In another terminal run `rosrun motion_plan obstacle_avoidance.py _x_goal:=<x> _y_goal:=<y>`.
   The node will drive the robot toward the goal coordinates while avoiding obstacles.

## Goal Based Navigation

`obstacle_avoidance.py` now subscribes to `/odom` so that it knows the robot pose. The goal
position can be provided through the private parameters `x_goal` and `y_goal`. The node
computes a velocity command that steers the robot toward the goal while still using the
original obstacle detection logic. When the robot reaches the goal (within 0.2&nbsp;m), it
stops automatically.

## Video

[![obstacle-avoidance-bot-using-ros-and-gazebo](https://img.youtube.com/vi/0pR6Cr7dMHA/0.jpg)](https://youtu.be/0pR6Cr7dMHA "obstacleAvoidanceRobot")  

## References
* [Video Series- Construct](https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1)
* [Statement motivation](https://github.com/udit7395/Obstacle-Avoidance-Using-ROS-And-Gazebo)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
