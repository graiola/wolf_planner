# WoLF planner (WIP)

## Installation:

install the required dependencies and clone the necessary repos in a catkin workspace:

```
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev ros-${ROS_DISTRO}-pinocchio ros-${ROS_DISTRO}-hpp-fcl
# Clone OCS2 
git clone git@github.com:graiola/ocs2.git
# Clone ocs2_robotic_assets
git clone https://github.com/graiola/ocs2_robotic_assets.git
```

compile only the necesary ocs2 packages:

```
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization -DCMAKE_BUILD_TYPE=Release
```

## Create the robot urdf:

if you want to use a different robot you need first to generate the urdf model for it. For example, with spot:

```
rosrun wolf_description_utils create_urdf_model.sh -r spot -d /tmp/wolf_planner
```

## Execution:

launch the `wolf_controller` with:

```
roslaunch wolf_controller wolf_controller_bringup.launch
```

use the graphic interface to set the control mode to `EXT` in order to accept external references. Then launch the mpc planner with:

```
roslaunch wolf_planner launch_planner.launch
```
