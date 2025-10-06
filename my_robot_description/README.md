# MY ROBOT DESCRIPTION PACKAGE

This package contains the URDF description of a custom 5R robot

The package contains three different launch files to launch the robot with:

- **RVIZ visualization and a fake GUI for publishing joint values, but no controllers**

```
ros2 launch my_robot_description robot_visualization.launch.py
```

- **RVIZ and Gazebo, but no controllers**

```
ros2 launch my_robot_description robot_gazebo_nocontrol.launch.py
```

- **RVIZ, Gazebo, and robot controllers**

```
ros2 launch my_robot_description robot_gazebo.launch.py
```

## USAGE AND DOCUMENTATION

Check ROS2 Controllers info: https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/doc/userdoc.rst

Check loaded controllers (active and inactive)
```
ros2 control list_controllers
```

Check topics of your controllers and their message types
```
ros2 topic list
```
```
ros2 topic info /position_controller/commands 
```

Test position controller, example commands:
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.1, 0.0, 0.0, 0.0]"
```
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
```
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 1.57]"
```
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.2, 0.0]"
```

Switch controllers using the controller manager in rqt:
```
rqt
```

Test velocity controller, example commands:
```
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.0, 0.0, 0.0, 0.0]"
```
```
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
```
```
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.05, 0.0, 0.0]"
```

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively, you can add this line to your .bashrc file

