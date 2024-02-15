cmake_minimum_required(VERSION 3.0.2)
project(riccardo_giacchino_master_thesis)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED std_msgs sensor_msgs message_generation)
find_package(cartesian_interface REQUIRED)
find_package(xbot2 REQUIRED)
#find_package(XBotInterface REQUIRED)

# catkin_package(
# #    INCLUDE_DIRS include
# #  LIBRARIES package_first
# #    CATKIN_DEPENDS cartesian_interface roscpp rospy std_msgs
# #  DEPENDS system_lib
# )

add_message_files(
    FILES
    RollPitchController.msg
    CartesianController.msg
    Custom_torque.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  include/package_first
  ${cartesian_interface_INCLUDE_DIRS}
  ${xbot2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #${XBotInterface_INCLUDE_DIRS}
)

add_library(imp_controller SHARED src/cartesian_impedance_controller/cartesianimpedancecontroller.cpp)
add_dependencies(imp_controller ${PROJECT_NAME}_generate_messages)
target_link_libraries(imp_controller ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2 ${XBotInterface_LIBRARIES} xbot2::xbot2_ros_support)

add_library(stab_implementation SHARED src/cartesian_impedance_controller/stability_compensation.cpp)
add_dependencies(stab_implementation ${PROJECT_NAME}_generate_messages)
target_link_libraries(stab_implementation ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2 ${XBotInterface_LIBRARIES} xbot2::xbot2_ros_support)

install(TARGETS imp_controller stab_implementation
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(CartesianInterfaceSolverImpSolver SHARED src/cartesian_impedance_controller/cartesianimpedancesolver.cpp
    src/cartesian_impedance_controller/cartesio_ros_wrapper.h)
target_link_libraries(CartesianInterfaceSolverImpSolver imp_controller stab_implementation ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} ${XBotInterface_LIBRARIES})

install(TARGETS CartesianInterfaceSolverImpSolver
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


## Cartesian impedance controller plugin
if(${xbot2_FOUND})

    add_xbot2_plugin(controllermanager
        src/cartesian_impedance_controller/controllermanager.cpp)

    add_dependencies(controllermanager ${PROJECT_NAME}_generate_messages)

    target_link_libraries(controllermanager
        PRIVATE
        imp_controller
        stab_implementation
        xbot2::xbot2_ros_support)

    install(TARGETS controllermanager
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
endif()

