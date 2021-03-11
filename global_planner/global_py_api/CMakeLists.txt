cmake_minimum_required(VERSION 2.8.3)
project(global_py_api)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nav_core
  pluginlib
  roscpp
  geometry_msgs
  nav_msgs
  move_base_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_generation
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_service_files(FILES
  py_flag.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_py_api
  CATKIN_DEPENDS
   costmap_2d 
   geometry_msgs
   nav_core 
   pluginlib 
   roscpp 
   tf2 
   tf2_geometry_msgs 
   tf2_ros
   message_runtime
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(global_py_api src/global_py_api.cpp)
add_dependencies(global_py_api ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_py_api
    ${catkin_LIBRARIES}
    )


install(TARGETS global_py_api
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


