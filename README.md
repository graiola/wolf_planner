# WoLF planner

MPC planner for WoLF based on ocs2. 

<p align="center">
  <img src="https://github.com/graiola/wolf-setup/blob/master/docs/mpc.gif" width="250" height="185" />
</p>

This work is based on [ocs2_legged_robot](https://github.com/leggedrobotics/ocs2/tree/main/ocs2_robotic_examples/ocs2_legged_robot) and [legged_control](https://github.com/qiayuanl/legged_control.git) from `qiayuanl`.

## Installation:

Install the required dependencies and clone the necessary repos in a catkin workspace:

```
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev ros-${ROS_DISTRO}-pinocchio ros-${ROS_DISTRO}-hpp-fcl libmpfr-dev
# Clone OCS2 
git clone git@github.com:graiola/ocs2.git
# Clone ocs2_robotic_assets
git clone https://github.com/graiola/ocs2_robotic_assets.git
```

Compile only the necesary ocs2 packages:

```
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization -DCMAKE_BUILD_TYPE=Release
```
For the wolf dependencies please refer to [wolf-setup](https://github.com/graiola/wolf-setup).

## Create the robot urdf:

If you want to use a different robot you need first to generate the urdf model for it. For example, with spot:

```
rosrun wolf_description_utils create_urdf_model.sh -r spot -d /tmp/wolf_planner
```

## Execution:

launch the `wolf_planner` and the `wolf_controller` with:

```
roslaunch wolf_planner launch_planner.launch
```

Stand up the robot by pressing `Enter` on the virtual keyboard or by using the RVIZ Wolf panel. Once the robot is up, change the control mode to `EXT` by pressing `Spacebar` on the virtual keyboard. At this point, the MPC will be connected with the whole-body controller and ready to use. To move the robot around, use the virtual keyboard. You can use the terminal commands to change the gait; use the `list` command to see what available gaits are.
