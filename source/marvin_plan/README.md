# How to use

## Setup
Steps 1 and 2 are also decribed in project README.md
1. Open a terminal, navigate to the source folder and run.
    ```
    catkin build
    source /opt/ros/noetic/setup.bash
    source /devel/setup.bash
    ```

2. Run **gazebo, rviz and topic publisher** using
    ```
    roslaunch marvin_control marvin_skid_teleop.launch
    ```
    or
    ```
    roslaunch carbot_gazebo_control carbot_gazebo_control.launch 
    ```

3. Run the node for planning
    ```
    rosrun marvin_plan hermite_curves_movement.py
    ```

4. Define the array of points inside the script

Robot goes through the curve and stopped at the end

**! By default node is using topics for skid steering. If you need to use it for ackermann steering you need to change corresponding publisher and subscriber topics** 

## Examples

For visualizing how the actual curve will look like the following tool was used: https://www.desmos.com/calculator/5knm5tkr8m

### Curve
![](../../docs/viz1.png){width=40% height=40%}

### Demo
![](../../docs/hermit1.mov)

#### Curve
<img src="../../docs/viz2.png" style="height:40%; width:40%">

#### Demo
![](../../docs/hermit2.mov)

#### Curve 
<img src="../../docs/viz3.png" style="height:40%; width:40%">

#### Demo
<!-- video3 -->
![](../../docs/hermit2.mov)

# Flow

* rospy was used for writing the script
* different hyperparameters (factors) were used and the best were remained
