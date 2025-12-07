# Sensor Based Control project of a robust control of a two wheeled mobile robot with kinematic disturbances

## Requirements

- **ROS2** (Humble / Foxy depending on your setup)  
- **TurtleBot3 packages**  
- **Gazebo** simulation environment  
- A **TurtleBot3 Burger** (for real-world execution)

Make sure the TurtleBot3 environment variables are correctly set.

## To launch the demo on Gazebo:

On terminal 1: 

```bash
`export TURTLEBOT3_MODEL=burger`

`ros2 launch turtlebot3_gazebo empty_world.launch.py`

```

On terminal 2:
```bash
`ros2 run turtlebot2_controler controller_node`
```


## To launch on turtlebot:

On terminal 1:
```bash
`export TURTLEBOT3_MODEL=burger`

`ros2 launch turtlebot3_bringup robot.launch.py`
```

On terminal 2:

```bash
`cd ~/turtlebot_ws`

`source install/setup.bash`

`ros2 run turtlebot2_controller controller_node`
```

### Results



![TurtleBot3 simulation with k1=1, k2=3 and k3=2](images/result_1.png)



![TurtleBot3 simulation with k1=22, k2=140 and k3=0.14](images/result_2.png)

### Notes

All text formatted as `this` represents commands to be entered in your terminal.

Ensure your workspace is built with:

```bash
colcon build
```