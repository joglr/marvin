# How to

This is a simple tutorial walking through how to get Marvin to be controlled by user's keyboard.

1. Have `marvin_gazebo, marvin_control, marvin_description` in your catkin workspace
1. Build the catkin workspace with `catkin build` (or `sudo catkin build` if you are running ROS as a Docker container)
1. Once the workspace builds you will need to have two terminal windows open, one for the teleop control and one for launching Gazebo
1. In one of these terminals run the following: `roslaunch marvin_control marvin_skid_teleop.launch` this will launch the following:
    - rviz which will show the positions of the robot in real time
    - gazebo which will be running the simulation
    - start publishing to topic `marvin/cmd_vel`
1. When gazebo is launched, open the second terminal window mentioned in step 3. There you want to type `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/marvin/cmd_vel`
    - You will know this is successfull if you see small table with explanations for control of the vehicle.
