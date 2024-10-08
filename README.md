# Master_Thesis

This repository contains the code developed by the master's thesis student, Riccardo Giacchino, for the development of a 
control strategy for the lower part of the robot CENTAURO to execute wheeled locomotion on uneven and rough terrain.

**NOTE**: **The branch `on_the_real_robot` is the most updated one. Please use it instead of the main one.**

## Introduction

This thesis proposes the design of two controllers for the wheel-on-leg robot CENTAURO, developed in the laboratory of Humanoid and Human-Centered Mechatronics 
at the Istituto Italiano di Tecnologia. The existing real-time framework was expanded by two modules: a Cartesian Impedance Controller and an Attitude Controller 
for each leg. The purpose of the former is to handle all the disturbances coming from the environment in a compliant way. When deviating from the reference values, 
this controller will generate a force to counteract the movement, following a mass-spring-damper dynamics. Tuning the parameters of the controller will change the 
behaviour of the robot to better match the condition of the terrain. The latter has been developed with the intent to dynamically change the reference values of the 
Cartesian impedance controller to control the roll and pitch angle of the base, maintaining the robot base parallel to the ground. The modified framework is then 
tested on both real and simulated scenarios to validate the effectiveness of the controllers.





 
  


 
