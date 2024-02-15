string name

geometry_msgs/Pose reference_position
geometry_msgs/Pose real_position
geometry_msgs/Twist position_error

geometry_msgs/Twist reference_velocity
geometry_msgs/Twist real_velocity
geometry_msgs/Twist velocity_error

geometry_msgs/Twist reference_acceleration

geometry_msgs/Wrench force

float64[46] torque

float64[6] K_omega

float64[6] inertia_op
