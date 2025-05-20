cmake_minimum_required(VERSION 3.10)
project(wolf_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        angles
        wolf_msgs
        wolf_planner_interface
        wolf_controller_utils
        )

find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)

catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            angles
            wolf_msgs
            wolf_planner_interface
            wolf_controller_utils
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
    ${hpp-fcl_INCLUDE_DIRS}
    ${ocs2_legged_robot_INCLUDE_DIRS}
    ${ocs2_legged_robot_ros_INCLUDE_DIRS}
    ${ocs2_self_collision_visualization_INCLUDE_DIRS}
    ${ocs2_sphere_approximation_INCLUDE_DIRS}
)

link_directories(
    ${pinocchio_LIBRARY_DIRS}
)

# Target trajectories publish node
add_executable(wolf_planner_target_node src/TargetTrajectoriesPublisher.cpp)
add_dependencies(wolf_planner_target_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wolf_planner_target_node
    ${catkin_LIBRARIES}
    ${ocs2_legged_robot_LIBRARIES}
    ${ocs2_legged_robot_ros_LIBRARIES}
    ${ocs2_self_collision_visualization_LIBRARIES})
target_compile_options(wolf_planner_target_node PRIVATE ${OCS2_CXX_FLAGS})

# Gait command node
add_executable(wolf_planner_gait_node src/GaitCommandNode.cpp)
add_dependencies(wolf_planner_gait_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wolf_planner_gait_node
    ${catkin_LIBRARIES}
    ${ocs2_legged_robot_LIBRARIES}
    ${ocs2_legged_robot_ros_LIBRARIES}
    ${ocs2_self_collision_visualization_LIBRARIES})
target_compile_options(wolf_planner_gait_node PRIVATE ${OCS2_CXX_FLAGS})

# RT Gui command node
add_executable(wolf_planner_rt_gui_node src/RtGuiGaitPublisher.cpp src/RtGuiCommandNode.cpp)
add_dependencies(wolf_planner_rt_gui_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wolf_planner_rt_gui_node
    ${catkin_LIBRARIES}
    ${ocs2_legged_robot_LIBRARIES}
    ${ocs2_legged_robot_ros_LIBRARIES}
    ${ocs2_self_collision_visualization_LIBRARIES})
target_compile_options(wolf_planner_rt_gui_node PRIVATE ${OCS2_CXX_FLAGS})

# MPC library
add_library(wolf_planner src/WolfPlannerRos.cpp)
add_dependencies(wolf_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(wolf_planner
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
    ${hpp-fcl_LIBRARIES}
    ${ocs2_legged_robot_LIBRARIES}
    ${ocs2_legged_robot_ros_LIBRARIES}
    ${ocs2_self_collision_visualization_LIBRARIES}
    ${ocs2_sphere_approximation_INCLUDE_LIBRARIES})

# MPC node
add_executable(wolf_planner_node src/WolfPlannerNode.cpp)
target_link_libraries(wolf_planner_node wolf_planner)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS wolf_planner wolf_planner_node wolf_planner_target_node wolf_planner_gait_node wolf_planner_rt_gui_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    PATTERN ".svn" EXCLUDE
  )
