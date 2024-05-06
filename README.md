# Guide to running the project

This is a simple tutorial walking through how to start a Gazebo simulation of Marvin, controlled via the keyboard.
It assumes that `marvin_gazebo, marvin_control, marvin_description` are in your catkin workspace

It works on Windows via. WSL 2 (Ubuntu) or on PopOS and MacOS.

## Steps

1. [Install **docker**](<Install docker.md>)
1. **Build** the docker image by running
    ```
    ./build.sh
    ```
1. To create and run the `my_ros` container, run the script:
    ```bash
    ./run.sh
    ```
    **Note:** This will stop and recreate the container, if it already exists

1. **Initialize and build the project** by running the following commands:
    ```bash
    cd ~/catkin_ws/
    sudo catkin build
    source ./devel/setup.bash
    ```

1. Run **gazebo, rviz and topic publisher** using
    ```
    roslaunch marvin_control marvin_skid_teleop.launch
    ```
    This will start...
    - **rviz** which will show the positions of the robot in real time
    - **gazebo** which will be running the simulation
    - **publishing** to topic `marvin/cmd_vel`

1. Now you need a **second terminal** instance for running **teleop control**, either via a TMUX session or by opening a new terminal and running
    ```bash
    ./connect.sh
    ```

1. To **initialize the second terminal**, you don't need to rebuild, just run:
    ```bash
    cd ~/catkin_ws/
    source ./devel/setup.bash
    ```

1. To **start the teleop control**, run:

    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/marvin/cmd_vel
    ```
    You will know this is successfull if you see small table with explanations for control of the vehicle.
