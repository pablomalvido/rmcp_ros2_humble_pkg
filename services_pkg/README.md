# SERVICES PACKAGE

This package contains examples of the usage of ROS2 Services.

It contains two nodes: number_service_server (which provides a service that says if a number is odd or even /check_even_odd), and number_service_client (which makes a request to this service).

## USAGE

These nodes can be run separately in two terminals:

**number_service_server**

```
ros2 run services_pkg number_service_server
```

**number_service_client**

```
ros2 run services_pkg number_service_client <input number>
```

## INTERESTING COMMANDS

Show all the available services:
```
ros2 service list
```

Call a service:
```
ros2 service call <service_name> <service_type> <arguments>
```

## REMINDER

Remember to source the workspace in every terminal you use

```
source install/setup.bash
```

Alternatively, you can add this line to your .bashrc file

