cmake_minimum_required(VERSION 3.10)
project(wolf_planner_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pluginlib
        wolf_msgs
        wolf_controller_utils
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)

catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            pluginlib
            wolf_msgs
            wolf_controller_utils
        DEPENDS
            pinocchio
            hpp-fcl
            ocs2_legged_robot
            ocs2_legged_robot_ros
            ocs2_self_collision
            ocs2_self_collision_visualization
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${hpp-fcl_INCLUDE_DIRS}
        ${ocs2_legged_robot_INCLUDE_DIRS}
        ${ocs2_legged_robot_ros_INCLUDE_DIRS}
        ${ocs2_self_collision_INCLUDE_DIRS}
        ${ocs2_self_collision_visualization_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
        src/constraint/EndEffectorLinearConstraint.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/constraint/ZeroVelocityConstraintCppAd.cpp
        src/initialization/LeggedRobotInitializer.cpp
        src/DefaultPlanner.cpp
        src/SwingTrajectoryPlanner.cpp
        src/LeggedReferenceManager.cpp
        src/LeggedRobotPreComputation.cpp
        src/LeggedInterface.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${hpp-fcl_LIBRARIES}
        ${ocs2_legged_robot_LIBRARIES}
        ${ocs2_legged_robot_ros_LIBRARIES}
        ${ocs2_self_collision_LIBRARIES}
        ${ocs2_self_collision_visualization_LIBRARIES}
        )
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(FILES wolf_planner_default_plugins.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
