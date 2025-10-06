# MY ROBOT MOVEIT CONFIG PACKAGE

This package contains the MoveIt configuration for our custom 5R robot

## USAGE AND DOCUMENTATION

Launch the robot with MoveIt controllers

```
ros2 launch my_robot_moveit_config demo_with_controllers.launch.py
```

Use the interactive marker of rviz to move the robot to different goal states, then plan and execute the trajectories

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively, you can add this line to your .bashrc file
