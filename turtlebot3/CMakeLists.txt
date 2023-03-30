cmake_minimum_required(VERSION 3.0.2)
<<<<<<< HEAD
project(turtlebot3)
=======
<<<<<<< HEAD:pp_msgs/CMakeLists.txt
project(pp_msgs)
=======
project(turtlebot3)
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5:turtlebot3/CMakeLists.txt
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
<<<<<<< HEAD
=======
<<<<<<< HEAD:pp_msgs/CMakeLists.txt
  message_generation
  roscpp
  std_msgs
=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
  actionlib
  nav_core
  nav_msgs
  tf
  base_local_planner
  move_base_msgs
  costmap_2d
  dynamic_reconfigure
  roscpp
  std_msgs
  pp_msgs
  message_generation
<<<<<<< HEAD
=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5:turtlebot3/CMakeLists.txt
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
)

## System dependencies are found with CMake's conventions
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
<<<<<<< HEAD
add_message_files(
  FILES
  Points.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
=======
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PathPlanningPlugin.srv
)
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
<<<<<<< HEAD
  geometry_msgs
  std_msgs
)
=======
  std_msgs
)

>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
<<<<<<< HEAD
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
=======
<<<<<<< HEAD:pp_msgs/CMakeLists.txt
#  INCLUDE_DIRS include
#  LIBRARIES pp_msgs
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
=======
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5:turtlebot3/CMakeLists.txt
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
<<<<<<< HEAD
 include
  ${catkin_INCLUDE_DIRS}
)

=======
# include
  ${catkin_INCLUDE_DIRS}
)

<<<<<<< HEAD:pp_msgs/CMakeLists.txt
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pp_msgs.cpp
# )

=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
## Declare a C++ library  
add_library(global_planner_lib 
  src/global_planner.cpp
)
<<<<<<< HEAD
=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5:turtlebot3/CMakeLists.txt
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
<<<<<<< HEAD
add_executable(bfs_server src/bfs_server.cpp src/bfs.cpp)
=======
<<<<<<< HEAD:pp_msgs/CMakeLists.txt
# add_executable(${PROJECT_NAME}_node src/pp_msgs_node.cpp)
=======
add_executable(bfs_server src/bfs_server.cpp)
add_executable(bfs src/bfs.cpp)
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5:turtlebot3/CMakeLists.txt
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(bfs_server PROPERTIES LINKER_LANGUAGE FIXED)

# set_target_properties(bfs PROPERTIES LINKER_LANGUAGE FIXED)

# set_target_properties(controller PROPERTIES LINKER_LANGUAGE FIXED)

## Add cmake target dependencies of the executable
## same as for the library above
<<<<<<< HEAD
add_dependencies(bfs_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
=======
<<<<<<< HEAD:pp_msgs/CMakeLists.txt
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
=======
add_dependencies(bfs_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bfs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5

## Specify libraries to link a library or executable target against
target_link_libraries(bfs_server
  ${catkin_LIBRARIES}
)
<<<<<<< HEAD
=======
target_link_libraries(bfs

  ${catkin_LIBRARIES}
)
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5:turtlebot3/CMakeLists.txt

>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

<<<<<<< HEAD
install(TARGETS bfs_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
=======

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5

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
<<<<<<< HEAD
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scripts.cpp)
=======
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pp_msgs.cpp)
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
