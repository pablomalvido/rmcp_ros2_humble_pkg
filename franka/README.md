# FRANKA PACKAGES

## Packages description

- **cartesian_controllers:** Includes a set of ROS2 cartesian controllers
- **python_scripts_pkg:** Includes a Python script to spawn an Interactive Marker to provide the target pose to a cartesian motion controller
- **franka_description:** Includes the URDF description of a Franka robot
- **custom_fr3_moveit_config:** MoveIt configuration package of a Franka robot
- **moveit_tests_pkg:** Contains scripts to move the Franka robot in MoveIt using the C++ API

## Usage

### MoveIt C++ API

Launch the robot and MoveIt controllers

```
ros2 launch custom_fr3_moveit_config demo.launch.py
```

Move the robot to target poses using the C++ API

```
ros2 run moveit_tests_pkg move_target_program
```

Or move the robot to target poses using cartesian straight motions with the C++ API

```
ros2 run moveit_tests_pkg move_cartesian_program
```

### MoveIt and Cartesian controllers

Launch the robot with MoveIt controllers (active) and a cartesian motion controller (inactive)

```
ros2 launch custom_fr3_moveit_config cartesian_controller.launch.py
```

Test MoveIt controller, move the robot in RVIZ using the MoveIt plugin

Switch controllers: Open the rqt controller manager 

```
rqt
```

And the go to Plugins/Robot Tools/Controller Manager

Deactivate fr3_arm_controller and activate cartesian_motion_controller

Test the cartesian controller, move with the Interactive marker

