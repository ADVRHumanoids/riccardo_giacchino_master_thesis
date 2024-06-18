## List of problem that should be resolved

- The stability controller works when the velocity and acceleration are inverted. Good luck finding why;
- In the `cartesianimpedancecontroller.cpp` source file, the computation of the angular velocity error is probably incorrectly computed. Refer to the Siciliano book for the correct formulation of the angular velocity error. The angular error instead should be correct since it is based on the Siciliano book.
- Lack of comment in the `cartesio_ros_wrapper` class. Please ask @Arturo Laurenzi for explanation.   
