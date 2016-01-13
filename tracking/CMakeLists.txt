cmake_minimum_required(VERSION 2.8.3)
project(tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_clustering
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PoseWithCovarianceStampedArray.msg 
   BoolStampedArray.msg
   Candidate.msg
   CandidateArray.msg 
   Int16StampedArray.msg
   Int32StampedArray.msg
   Int64StampedArray.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   EmptyLikelihood.srv
   EmptyService.srv
   SaveCountingMap.srv
   SetLaserUsed.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

)

## Declare a cpp library
# add_library(tracking
#   src/${PROJECT_NAME}/tracking.cpp
# )

## Declare a cpp executable

 add_executable(preprocess_cluster src/preprocess_cluster.cpp src/my_algebra_common.cpp)
 add_executable(compute_likelihood src/compute_likelihood.cpp src/my_algebra_common.cpp) 
 add_executable(compute_likelihood_detection src/compute_likelihood_detection.cpp src/my_algebra_common.cpp) 
 add_executable(compute_likelihood_tracking src/compute_likelihood_tracking.cpp src/my_algebra_common.cpp) 
 add_executable(show_likelihood src/show_likelihood.cpp)
 add_executable(tracking_particle_filter src/tracking_particle_filter.cpp src/my_algebra_common.cpp)

 add_executable(laser_visit_map src/laser_visit_map.cpp)
 add_executable(convert_laser_visit_map src/convert_laser_visit_map.cpp)
 add_executable(affordance_map src/affordance_map.cpp)
 add_executable(sync_visit_maps src/sync_visit_maps.cpp)
 add_executable(build_map src/build_map.cpp)
 add_executable(density_map src/density_map.cpp)
 add_executable(cumulative_density_map src/cumulative_density_map.cpp)
 add_executable(laser_visit_map_for_mapping src/laser_visit_map_for_mapping.cpp)
 
 add_executable(velodyne_transformer src/velodyne_transformer.cpp)


 add_executable(on_board_human_tracker_2d src/old_code/on_board_human_tracker_2d.cpp src/my_algebra_common.cpp)
 add_executable(on_board_human_tracker_2d_likelihood src/old_code/on_board_human_tracker_2d_likelihood.cpp src/my_algebra_common.cpp)
 add_executable(build_costmap_value src/old_code/build_costmap_value.cpp)
 
 add_executable(fake_publisher src/old_code/fake_publisher.cpp)
 add_executable(fake_cluster src/fake_cluster.cpp)
 add_executable(fake_humans src/old_code/fake_humans.cpp)
 add_executable(fake_cloud_laser src/fake_cloud_laser.cpp)
 #add_executable(density_map_debug src/old_code/density_map_debug.cpp)
 
# add_executable(build_costmap_value_run_time src/build_costmap_value_run_time.cpp)
# add_executable(raffa_tracking_particle_filter src/raffa_tracking_particle_filter.cpp src/my_algebra_common.cpp)


 add_dependencies(build_map tracking_generate_messages_cpp)
 add_dependencies(on_board_human_tracker_2d laser_clustering_generate_messages_cpp)
 add_dependencies(on_board_human_tracker_2d_likelihood laser_clustering_generate_messages_cpp)
 add_dependencies(preprocess_cluster laser_clustering_generate_messages_cpp tracking_generate_messages_cpp)
 add_dependencies(compute_likelihood laser_clustering_generate_messages_cpp tracking_generate_messages_cpp)
 add_dependencies(compute_likelihood_detection laser_clustering_generate_messages_cpp tracking_generate_messages_cpp)
 add_dependencies(compute_likelihood_tracking laser_clustering_generate_messages_cpp tracking_generate_messages_cpp)
 add_dependencies(show_likelihood tracking_generate_messages_cpp)
 add_dependencies(tracking_particle_filter tracking_generate_messages_cpp)
 add_dependencies(affordance_map  tracking_generate_messages_cpp)
 add_dependencies(sync_visit_maps tracking_generate_messages_cpp)

 add_dependencies(fake_humans tracking_generate_messages_cpp)
 add_dependencies(fake_cluster tracking_generate_messages_cpp)

 add_dependencies(convert_laser_visit_map tracking_generate_messages_cpp)
 add_dependencies(density_map tracking_generate_messages_cpp)
 add_dependencies(laser_visit_map tracking_generate_messages_cpp)
 add_dependencies(laser_visit_map_for_mapping tracking_generate_messages_cpp)

 

# add_dependencies(human_tracked_map tracking_generate_messages_cpp)
# add_dependencies(raffa_tracking_particle_filter tracking_generate_messages_cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(tracking_node tracking_generate_messages_cpp)

## Specify libraries to link a library or executable target against

 target_link_libraries(on_board_human_tracker_2d
   ${catkin_LIBRARIES}
 )
 target_link_libraries(on_board_human_tracker_2d_likelihood
   ${catkin_LIBRARIES}
 )
 target_link_libraries(preprocess_cluster
   ${catkin_LIBRARIES}
 )
 target_link_libraries(compute_likelihood
   ${catkin_LIBRARIES}
 )

 target_link_libraries(compute_likelihood_detection
   ${catkin_LIBRARIES}
 )

 target_link_libraries(compute_likelihood_tracking
   ${catkin_LIBRARIES}
 )

 target_link_libraries(show_likelihood
   ${catkin_LIBRARIES}
 )
 target_link_libraries(tracking_particle_filter
   ${catkin_LIBRARIES}
)

 target_link_libraries(laser_visit_map
   ${catkin_LIBRARIES}
)

 target_link_libraries(convert_laser_visit_map
   ${catkin_LIBRARIES}
)

 target_link_libraries(laser_visit_map_for_mapping
   ${catkin_LIBRARIES}
)


 target_link_libraries(affordance_map
   ${catkin_LIBRARIES}
)
 target_link_libraries(density_map
   ${catkin_LIBRARIES}
)

 target_link_libraries(cumulative_density_map
   ${catkin_LIBRARIES}
)

 target_link_libraries(build_costmap_value
   ${catkin_LIBRARIES}
)

 target_link_libraries(build_map
   ${catkin_LIBRARIES}
)


 target_link_libraries(sync_visit_maps
   ${catkin_LIBRARIES}
)


 target_link_libraries(fake_publisher
   ${catkin_LIBRARIES}
)

 target_link_libraries(fake_humans
   ${catkin_LIBRARIES}
)

 target_link_libraries(fake_cloud_laser
   ${catkin_LIBRARIES}
)

 target_link_libraries(fake_cluster
   ${catkin_LIBRARIES}
)

# target_link_libraries(density_map_debug
#   ${catkin_LIBRARIES}
#)


 target_link_libraries(velodyne_transformer
   ${catkin_LIBRARIES}
)






#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tracking tracking_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
