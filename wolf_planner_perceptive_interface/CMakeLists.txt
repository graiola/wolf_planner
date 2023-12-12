cmake_minimum_required(VERSION 3.10)
project(wolf_planner_perceptive_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        wolf_planner_interface
        convex_plane_decomposition_msgs
        convex_plane_decomposition_ros
        grid_map_sdf
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(pinocchio REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_sphere_approximation REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        wolf_planner_interface
        convex_plane_decomposition_msgs
        convex_plane_decomposition_ros
        grid_map_sdf
        DEPENDS
        pinocchio
)

###########
## Build ##
###########

#set(FLAGS
#        ${OCS2_CXX_FLAGS}
#        ${pinocchio_CFLAGS_OTHER}
#        -Wno-ignored-attributes
#        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
#        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
#        )


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${ocs2_legged_robot_INCLUDE_DIRS}
        ${ocs2_sphere_approximation_INCLUDE_DIRS}
        ${ocs2_centroidal_model_INCLUDE_DIRS}
        ${ocs2_self_collision_INCLUDE_DIRS}
        ${ocs2_ros_interfaces_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
        src/constraint/FootCollisionConstraint.cpp
        src/constraint/FootPlacementConstraint.cpp
        src/constraint/SphereSdfConstraint.cpp
        src/visualization/FootPlacementVisualization.cpp
        src/visualization/SphereVisualization.cpp
        src/synchronized_module/PlanarTerrainReceiver.cpp
        src/ConvexRegionSelector.cpp
        src/PerceptiveLeggedReferenceManager.cpp
        src/PerceptiveLeggedPrecomputation.cpp
        src/PerceptiveLeggedInterface.cpp
        src/PerceptiveController.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${ocs2_legged_robot_LIBRARIES}
        ${ocs2_sphere_approximation_LIBRARIES}
        ${ocs2_centroidal_model_LIBRARIES}
        ${ocs2_self_collision_LIBRARIES}
        ${ocs2_ros_interfaces_LIBRARIES}
        )

#target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

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
