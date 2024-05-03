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
