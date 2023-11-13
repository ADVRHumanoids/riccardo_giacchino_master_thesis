cmake_minimum_required(VERSION 3.0.2)
project(package_first)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(cartesian_interface REQUIRED)
find_package(xbot2 REQUIRED)
find_package(awesome_utils REQUIRED)
#find_package(XBotInterface REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES package_first
#  CATKIN_DEPENDS cartesian_interface roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/${PROJECT_NAME}
  ${cartesian_interface_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${xbot2_INCLUDE_DIRS}
 # ${XBotInterface_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/motiongenerator.cpp
                            src/basestategetter.cpp
                            src/emafilter.cpp
                            src/lowpassfilter.cpp
                            src/grahamscanconvexhull.cpp
                            src/cartesian_impedance_controller/cartesianimpedancecontroller.cpp
                            src/cartesian_impedance_controller/controllermanager.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils)

## First project - setup_node
add_executable(setup_node src/setup.cpp)
target_link_libraries(setup_node ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

## Second project - leg_motion
add_executable(leg_motion_node src/leg_motion.cpp)
target_link_libraries(leg_motion_node ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

## Third project - robot motion
add_executable(moving_model_node src/moving_model.cpp)
target_link_libraries(moving_model_node ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

## Forth project - base motion
add_executable(base_motion_node src/base_motion.cpp)
target_link_libraries(base_motion_node ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

## Seventh project - PI controller for velocity
add_executable(vel_control_node src/velocity_controller_node.cpp)
target_link_libraries(vel_control_node ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} ${xbot2_LIBRARIES})

## Sixth project - COM position and projection
add_executable(stability_node src/stability_node.cpp)
target_link_libraries(stability_node
                        ${catkin_LIBRARIES}
                        ${cartesian_interface_LIBRARIES}
                        ${xbot2_LIBRARIES}
                        ${PROJECT_NAME}
                        awesome_utils::awesome_utils)

## Seventh project - raise one leg of the robot
add_executable(raise_one_leg_node src/raise_one_leg.cpp)
target_link_libraries(raise_one_leg_node
                        ${catkin_LIBRARIES}
                        ${cartesian_interface_LIBRARIES}
                        ${xbot2_LIBRARIES}
                        ${PROJECT_NAME}
                        awesome_utils::awesome_utils)

add_executable(cartesian_imp_controller_node src/cartesian_impedance_controller/cic_main.cpp)
target_link_libraries(cartesian_imp_controller_node
                        ${catkin_LIBRARIES}
                        ${cartesian_interface_LIBRARIES}
                        ${xbot2_LIBRARIES}
                        ${PROJECT_NAME}
                        awesome_utils::awesome_utils)


install(TARGETS stability_node
                vel_control_node
                raise_one_leg_node
                base_motion_node
                cartesian_imp_controller_node
                ${PROJECT_NAME}

   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

