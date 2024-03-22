cmake_minimum_required(VERSION 3.0.2)
project(my_lqr_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  gazebo_ros_control
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  multi_msgs
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(cmake_modules REQUIRED)
find_package(tf REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(unitree_motorsdk_a1)
include_directories(
  /home/cherish/code/yuquan/sim_wheel_ball/src/unitree_motorsdk_a1/include
)




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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
  INCLUDE_DIRS include
#  LIBRARIES my_lqr_controller
#  CATKIN_DEPENDS controller_interface gazebo_ros_control roscpp std_msgs
#  DEPENDS system_lib
   DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_lqr_controller.cpp
# )

## 声明C++库
add_library(head_lib
  include/my_lqr_controller/lqr_control.h
  src/lqr_control.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/my_lqr_controller_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_lqr_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_dependencies(head_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head_lib
  ${catkin_LIBRARIES}
  /home/cherish/code/yuquan/sim_wheel_ball/src/unitree_motorsdk_a1/lib/libunitreeMotorSDK_Linux64.so
)

add_executable(use_head src/use_head.cpp)

add_dependencies(use_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#此处需要添加之前设置的 head 库
target_link_libraries(use_head
  head_lib
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)


#Generate the static library from the library sources
add_library(motor_lqr_control_lib
        include/my_lqr_controller/motor_lqr_control.h
        src/motor_lqr_control.cpp
        )

target_link_libraries(motor_lqr_control_lib
                      ${catkin_LIBRARIES}
                      /home/cherish/code/yuquan/sim_wheel_ball/src/unitree_motorsdk_a1/lib/libunitreeMotorSDK_Linux64.so
                      )

## a1_motor_number4_test.cpp test

add_executable(test_motor_lqr_standup src/test_motor_lqr_standup.cpp)

add_dependencies(test_motor_lqr_standup ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_motor_lqr_standup
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
 /home/cherish/code/yuquan/sim_wheel_ball/src/unitree_motorsdk_a1/lib/libunitreeMotorSDK_Linux64.so
 motor_lqr_control_lib
 )



#  lqr_gazebo_controller
add_executable(lqr_gazebo_controller src/lqr_gazebo_controller.cpp)

add_dependencies(lqr_gazebo_controller ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lqr_gazebo_controller
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
 )

 #  gazebo_set_state.cpp
add_executable(gazebo_set_state src/gazebo_set_state.cpp)

add_dependencies(gazebo_set_state ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_set_state
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
 )


 #  gazebo_set_state.cpp
 add_executable(gazebo_leg_controller src/gazebo_leg_controller.cpp)

 add_dependencies(gazebo_leg_controller ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
 target_link_libraries(gazebo_leg_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

 #  gazebo_jump_controller.cpp
 add_executable(gazebo_jump_controller src/gazebo_jump_controller.cpp)

 add_dependencies(gazebo_jump_controller ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
 target_link_libraries(gazebo_jump_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# gazebo_vmc_controller.cpp
add_executable(gazebo_vmc_controller src/gazebo_vmc_controller.cpp)

add_dependencies(gazebo_vmc_controller ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_vmc_controller
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
 leg_kinematics_lib
 )

 ## 声明生成leg_kinematics库leg_kinematics.cpp
add_library(leg_kinematics_lib
include/my_lqr_controller/leg_kinematics.h
src/leg_kinematics.cpp
)
# target_link_libraries(motor_lqr_control_lib
#                       ${catkin_LIBRARIES}
#                       )


## test_leg_kinematics.cpp test

add_executable(test_leg_kinematics src/test_leg_kinematics.cpp)

add_dependencies(test_leg_kinematics ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_leg_kinematics
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
 leg_kinematics_lib
 )

#  gazebo_roll_control


add_executable(gazebo_roll_control src/gazebo_roll_control.cpp)

add_dependencies(gazebo_roll_control ${test_command_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_roll_control
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
 )