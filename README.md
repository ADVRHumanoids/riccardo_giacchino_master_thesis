# Master_Thesis

This repository contains the code developed by the master's thesis student, Riccardo Giacchino, for the development of a 
control strategy for the lower part of the robot CENTAURO to execute wheeled locomotion on uneven and rough terrain.

**NOTE**: The branch 'on_the_real_robot' is the most updated one. Please use it instead of the main one.


## Introduction

This thesis proposes the design of two controllers for the wheel-on-leg robot CENTAURO, developed in the laboratory of Humanoid and Human-Centered Mechatronics 
at the Istituto Italiano di Tecnologia. The existing real-time framework was expanded by two modules: a Cartesian Impedance Controller and an Attitude Controller 
for each leg. The purpose of the former is to handle all the disturbances coming from the environment in a compliant way. When deviating from the reference values, 
this controller will generate a force to counteract the movement, following a mass-spring-damper dynamics. Tuning the parameters of the controller will change the 
behavior of the robot to better match the condition of the terrain. The latter has been developed with the intent to dynamically change the reference values of the 
Cartesian impedance controller to control the roll and pitch angle of the base, maintaining the robot base parallel to the ground. The modified framework is then 
tested on both real and simulated scenarios to validate the effectiveness of the controllers.


## File System Structure

- ***/CMakeFiles*** contain the file generated by CMake    
- ***/CentauroConfig*** is the folder containing all the configuration file (.yaml) employed in the code for quick setup of the controllers as well as the Xbot2 and CartesI/O framework configuration    
- ***/documentation*** present the PDFs of the thesis, both summury and complete master thesis, which serves as a theoretical documentation for the implementation of the two controllers
- ***/include*** contains the header files for the created classes
- ***/msg*** is the folder containing all the custom messages created for ROS, which are needed for the data log in real-time
- ***/src*** is the heart of this repository, since it contains all the source files with the implementation of the controllers
   

## Configuration Files

This section present a description of each configuration files created for the setup of the complete framework.

### `new_centauro_config.yaml` 

This configuration file is used to set up the Xbot2 framework, following the guidelines outlined in its documentation.
Of particular importance here is the addition of the `impedance_control` plugin, which integrates the Cartesian Impedance Controller into Xbotcore.
The plugin receives parameters such as `stack_path`, which is the path to the stack of problem configuration files (to be further analyzed),
`stab_controller_path` for inserting the path to the Attitude Controller configuration file, and a boolean parameter `stab_control_enable` to quickly enable or disable
the presence of the Attitude Controller in the control strategy.


 
