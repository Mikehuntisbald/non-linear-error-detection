cmake_minimum_required(VERSION 3.0.2)
project(nuise)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
#        base_local_planner
#        clear_costmap_recovery
        cmake_modules
#        costmap_2d
#        dynamic_reconfigure
        geometry_msgs
        message_generation

#        nav_core
        nav_msgs
#        navfn
        pluginlib
        roscpp
        rospy
#        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
        tf
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/MoveBase.cfg
#)

catkin_package(

    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES nuise
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs

        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
#add_library(nuise
#    src/nuise.cpp
#    src/map_helper_ros.cpp
#)
add_executable(nuise
  src/nuise.cpp)

add_executable(map_helper_ros
        src/map_helper_ros.cpp)

target_link_libraries(nuise
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
target_link_libraries(map_helper_ros
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
add_dependencies(nuise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_helper_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(nuise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(nuise_node nuise)
#set_target_properties(nuise_node PROPERTIES OUTPUT_NAME nuise)

#install(
#    TARGETS
#        nuise_node
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(
    TARGETS
        nuise
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
