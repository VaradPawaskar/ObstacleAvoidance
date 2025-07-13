# Obstacle Avoidance Robot Simulation  

_Building a differential drive robot which can avoid obstacles 
This work was done taking reference from (https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1).

## Prerequisites  

* [ROS](http://wiki.ros.org/noetic)  
* [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs)

## Getting Started

1. Clone this repository.
2. Run `catkin_make` for both `catkin_ws` and `simulation_ws`.
3. Launch your terminal and run the command `roslaunch my_worlds <world_name>.launch`. 
This will launch the gazebo enviroment
4. In another terminal, run the command `roslaunch robot_description spawn.launch`. 
This will load the robot in the environment at origin. It can be spawned at different location by giving additional arguments like `x:=3 y:=8 z:=7`.  
5. In another terminal run `rosrun motion_plan obstacle_avoidance.py
6. In another terminal run 'rosrun motion_plan set_goal.py'
