### Sensor Based Control project of a robust control of a two wheeled mobile robot with kinematic disturbances

## To launch the demo on Gazebo:

On terminal 1: 

`export TURTLEBOT3_MODEL=burger`


`ros2 launch turtlebot3_gazebo empty_world.launch.py`

On terminal 2:

`ros2 run turtlebot2_controler controller_node`


## To launch on turtlebot:

On terminal 1:

`export TURTLEBOT3_MODEL=burger`

`ros2 launch turtlebot3_bringup robot.launch.py`

On terminal 2:

`cd ~/turtlebot_ws`

`source install/setup.bash`

`ros2 run turtlebot2_controller controller_node`