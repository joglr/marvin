# Marvin Control

This directory contains all relevant code for different types of steering that can be used with the robot. As of right now, only skid steering and Ackermann steering are successfully implemented.

## Skid steering

Skid steering is implemented via Gazebo's built-in module. To control, publish a Twist message to the `marvin/cmd_vel` topic. Below is a diagram showing how messages flow in the Ackermann model. The communication model is given below:

![Skid steering flow of information](./images/skid_steering_control.png)

For launching the skid steering model, run the following:

```bash
roslaunch marvin_control marvin_skid_teleop.launch
```

Then any publishing to the marvin/cmd_vel topic will control the robot. These are the following possibilities for control:

1. To **start the teleop control**, run:

    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/marvin/cmd_vel
    ```

    You will know this is successfull if you see small table with explanations for control of the vehicle.

or

1. For **starting the movement planner**, run:

    ```bash
    rosrun marvin_plan hermit_curves_movement.py
    ```

    You will know this is successfull if you see a message "Node has been started" and a description of targets will show

## Ackermann steering

## Swerve steering

## Work distribution

- Erik: Implemented skid steering, helped with debugging of Ackermann steering
- Jonas: Helped with debugging of skid steering
- Milan: Implementation of Ackermann steering
- Kirill: - 
