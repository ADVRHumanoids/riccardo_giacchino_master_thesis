# Master_Thesis

This repository contains the code developed by the master's thesis student, Riccardo Giacchino, for the development of a 
control strategy for the lower part of the robot CENTAURO to execute wheeled locomotion on uneven and rough terrain.

**NOTE**: The branch 'on_the_real_robot' is the most updated one. Please use it instead of the main one.


## Introduction

This thesis proposes the design of two controllers for the wheel-on-leg robot CENTAURO, developed in the laboratory of Humanoid and Human-Centered Mechatronics 
at the Istituto Italiano di Tecnologia. The existing real-time framework was expanded by two modules: a Cartesian Impedance Controller and an Attitude Controller 
for each leg. The purpose of the former is to handle all the disturbances coming from the environment in a compliant way. When deviating from the reference values, 
this controller will generate a force to counteract the movement, following a mass-spring-damper dynamics. Tuning the parameters of the controller will change the 
behaviour of the robot to better match the condition of the terrain. The latter has been developed with the intent to dynamically change the reference values of the 
Cartesian impedance controller to control the roll and pitch angle of the base, maintaining the robot base parallel to the ground. The modified framework is then 
tested on both real and simulated scenarios to validate the effectiveness of the controllers.


## File System Structure

- ***/CMakeFiles*** contain the file generated by CMake    
- ***/CentauroConfig*** is the folder containing all the configuration files (.yaml) employed in the code for quick setup of the controllers as well as the Xbot2 and CartesI/O framework configuration    
- ***/documentation*** present the PDFs of the thesis, both summary and complete master thesis, which serves as theoretical documentation for the implementation of the two controllers
- ***/include*** contains the header files for the created classes
- ***/msg*** is the folder containing all the custom messages created for ROS, which are needed for the data log in real-time
- ***/src*** is the heart of this repository since it contains all the source files with the implementation of the controllers
   

## Configuration Files

This section presents a description of each configuration file created for the setup of the complete framework.

### `new_centauro_config.yaml` 

This configuration file is used to set up the Xbot2 framework, following the guidelines outlined in its documentation.
Of particular importance here is the addition of the `impedance_control` plugin, which integrates the Cartesian Impedance Controller into Xbotcore.
The plugin receives parameters such as `stack_path`, which is the path to the stack of problem configuration files (to be further analyzed).
`stab_controller_path` is used to insert the path of the Attitude Controller configuration file, while the boolean parameter `stab_control_enable` quickly enables or disables
the presence of the Attitude Controller in the control strategy.

### `stack_problem.yaml`

This is the file used to craft the optimization problem that will be solved by the framework [CartesI/O](https://advrhumanoids.github.io/CartesianInterface/ "") a novel Cartesian control framework with a focus on online control of multi-chained, hyper-redundant floating-base robots. In this case, the defined problems are of the *Interaction Task* type, which contains the parameters about the virtual spring stiffness, damping ratio and the name of the two links (base and end-effector) limiting the kinematic chain. Please refer to the framework documentation for a proper description of how to write a stack of problems configuration file.

### `roll_pitch_controller_config.yaml`

The last config file has been custom-made for easy setting of the Attitude controller parameters. Each chain (in this case each leg) will be controlled by a single Attitude controller, divided into a pitch and roll impedance controller, whose parameters are read from this file. It contains the proportional gain values, as well as the damping factor, used to compute the corresponding derivative gain (also referred to as the velocity gain). Each impedance controller also features a parameter known as `relative_leg`. This parameter denotes the opposing leg to which the controller refers, essential for calculating the distance value utilized in determining the necessary control action. This is better explained in Chapter 5 of my [thesis](https://github.com/ADVRHumanoids/riccardo_giacchino_master_thesis/blob/on_the_real_robot/documentation/Riccardo_s_Master_Thesis_Final.pdf). Then the `Safety_limit` section set the values for some check in the code used to trigger an emergency stop in case of sudden and incorrect control action which may [occur](https://drive.google.com/file/d/1S7bSzhlrEtKGn1ROlpxgQXZMqdAXpGou/view?usp=sharing). They haven't been widely tested, and they work as badly as the Attitude controller itself.




 
  


 
