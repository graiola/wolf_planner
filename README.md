# WoLF planner (WIP)

## Installation:

clone the necessary repos in a catkin workspace:

```
# Clone OCS2
git clone git@github.com:graiola/ocs2.git
# Clone pinocchio
git clone --recurse-submodules https://github.com/graiola/pinocchio.git
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/graiola/hpp-fcl.git
# Clone ocs2_robotic_assets
git clone https://github.com/graiola/ocs2_robotic_assets.git
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
```

compile only the necesary ocs2 packages:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
```

## Create the robot urdf:

for example, with spot:

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
