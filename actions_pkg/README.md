# ACTIONS PACKAGE

This package contains examples of the usage of ROS2 Actions.

It contains two nodes: fibonacci_server (which provides an action that prints the fibonacci sequence of a certain order /fibonacci), and fibonacci_client (which sends a goal (the order of the sequence) to this action).

These nodes can be run separately in two terminals:

**fibonacci_server**

```
ros2 run actions_pkg fibonacci_server
```

**fibonacci_client**

```
ros2 run actions_pkg fibonacci_client
```

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively, you can add this line to your .bashrc file

