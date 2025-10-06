# TOPICS PACKAGE

This package contains examples of the usage of ROS2 Topics.

It contains two nodes: talker (which publishes a message into the /chatter topic every second), a listener (which subscribes to the /chatter topic and prints the messages publishes in it).

These nodes can be run separately in two terminals:

**Talker**

```
ros2 run topics_pkg talker
```

**Listener**

```
ros2 run topics_pkg listener
```

Or together in a single launch file:

```
ros2 launch topics_pkg talker_listener.launch.py
```

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively you can add this line to your .bashrc file
