cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        wolf_msgs
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        wolf_msgs
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

# Target trajectories publish node
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(legged_target_trajectories_publisher
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_target_trajectories_publisher
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

# MPC node
add_executable(ocs2_mpc
        src/Ocs2Mpc.cpp
        )
add_dependencies(ocs2_mpc
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ocs2_mpc
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ocs2_mpc legged_target_trajectories_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
