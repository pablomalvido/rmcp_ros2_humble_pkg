# ACTIONS PACKAGE

This package contains examples of the usage of ROS2 Actions.

It contains two nodes: fibonacci_server (which provides an action that prints the fibonacci sequence of a certain order /fibonacci), and fibonacci_client (which sends a goal (the order of the sequence) to this action).

## USAGE

These nodes can be run separately in two terminals:

**fibonacci_server**

```
ros2 run actions_pkg fibonacci_server
```

**fibonacci_client**

```
ros2 run actions_pkg fibonacci_client
```

## INTERESTING COMMANDS AND DOCUMENTATION

Show all the available actions:
```
ros2 action list
```

Check action status:

```
ros2 topic echo /fibonacci/_action/status
```

Action status meaning: https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html

Action lifecycle diagram: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively, you can add this line to your .bashrc file



