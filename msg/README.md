## ROS Custom Message 

You cannot understand why your controller is not working just by looking at the robot falling, sometimes you also need to look at a wave going up and down, which still doesn't help you to figure it out.
Here are reported the custom msg files crafted to extrapolate from the controller every single signal you can think of. 
It is fair to point out that the developed framework is real-time, therefore all signals will be published online to the corresponding topic and can be analysed through tools such as [Plotjuggler](https://plotjuggler.io/ "So you can see your control action diverge also on the screen ahah").
In particular, each msg file is used in only one of the created classes: 
- *CartesianController.msg* ->  *cartesianimpedancecontroller.h*
- *Custom_torque.msg* -> *controllermanager.h*
- *RollPitchController.msg* -> *stability_compensation.h*

Remember that being able to use this custom message file is not as simple as it seems. After creating the file in the corresponding msg folder, you will need to write some stuff in the [CMakeLists.txt](docs/CMakeLists.txt) file.
Add these lines to your file: 
```
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
```
then in the same file, when adding and installing the header and source file remember to add the dependency `add_dependencies(#executable_name ${PROJECT_NAME}_generate_messages)`, as reported in the following example
```
add_library(stab_implementation SHARED src/cartesian_impedance_controller/stability_compensation.cpp)
add_dependencies(stab_implementation ${PROJECT_NAME}_generate_messages)
target_link_libraries(stab_implementation ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2 ${XBotInterface_LIBRARIES} xbot2::xbot2_ros_support)

install(TARGETS imp_controller stab_implementation
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
```

Then, the most important one, run the command `cmake .`, followed by `make install`, to build and install the new dependencies.
