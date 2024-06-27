# Marvin Control

This directory contains all relevant code for different types of steering that can be used with the robot. 

## Skid steering

Skid steering was implemented via the use of the Gazebo's built-in functionalities. The biggest hurdle that we had to overcome was setting the inertia matrices correctly and ensuring that correct topics were being subscribed / published to. The flow of information is shown on the image below:

![Skid steering flow of information]()

For launching the skid steering model, run the following:

```bash
roslaunch marvin_control marvin_skid_teleop.launch
```

Then any publishing to the marvin/cmd_vel topic will control the robot. That is you can either control it via teleop or via planning with:

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
