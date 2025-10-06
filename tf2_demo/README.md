# TF2 PACKAGE

This package contains examples of the usage of ROS2 TF2 Library, which allows to keep track of multiple coordinate frames.

It contains two nodes: broadcaster (which publishes a new TF frame with respect to a robot frame), and listener (which prints the relative pose between two robot frames/links).

## USAGE

Launch the robot visualization in one terminal

```
ros2 launch my_robot_description robot_visualization.launch.py
```

Run the broadcaster in a different terminal

```
ros2 run tf2_demo broadcaster
```

Run the listener in a different terminal

```
ros2 run tf2_demo listener
```

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively, you can add this line to your .bashrc file



