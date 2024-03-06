#include "controllermanager.h"

// ==============================================================================
// Real-Time plugin required functions
// ==============================================================================

bool ControllerManager::on_initialize()
{
    _dt = getPeriodSec();
    _time = 0;

    _robot->sense();

    // Model creation
    _model = ModelInterface::getModel(_robot->getConfigOptions());
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    _model->update();

    // Take the path to the problem definition file
    _config_parameters_stab_controller = YAML::LoadFile(getParamOrThrow<string>("~stab_controller_path"));

    stab_controller_enable = getParamOrThrow<int>("~stab_control_enable");

    _ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<Parameters>(_dt),
                                     _model);

    ProblemDescription pb(YAML::LoadFile(getParamOrThrow<string>("~stack_path")),
                          _ctx);

    // Solver creation
    _solver = CartesianInterfaceImpl::MakeInstance("ImpSolver",
                                                   pb,
                                                   _ctx);

    // Obtaining the tasks from the problem description
    _tasks = pb.getTask(pb.getNumTasks()-1);

    // Extract task names
    get_task();

    // Create the joint map
    joint_map_generator();

    // Set control mode for the joints found in the previous function
    setDefaultControlMode(_ctrl_map);

    // ----------------------------------------------------------------------------- TODO: comments
    if(stab_controller_enable == true){
        stability_controller_initialization();
    }

    // Initialize usefull variable
    _torque_cartesian = _torque_contact = _torque = Eigen::VectorXd::Zero(_model->getJointNum());

    //
    _ros_wrapper = std::make_unique<CartesioRosWrapper>(_solver, "ci", "cartesian");

    // Initialize the gravity compensation term
    _gravity_torque = Eigen::VectorXd::Zero(_model->getJointNum());
    _g = Eigen::VectorXd::Zero(6);
    _J_c.resize(4, Eigen::MatrixXd::Identity(6, _model->getJointNum()));
    _J_cz = Eigen::MatrixXd::Identity(6, 4);
    _J_cz_pseudo_inverse = Eigen::MatrixXd::Identity(4, 6);
    _J_leg.resize(4, Eigen::MatrixXd::Identity(6, _model->getJointNum()));
    _contact_force_z = Eigen::VectorXd::Zero(4);

    // FIXME: Other
    _total_Jd_Qd = Eigen::VectorXd::Zero(46);

    // ============================== DEBUG ==============================
    // XBot::MatLogger2::Options opt;
    // opt.default_buffer_size = 1e9;
    // _logger = XBot::MatLogger2::MakeLogger("/home/riccardo/Documents/MATLAB/logger.mat", opt);
    // _imu = _model->getImu("pelvis");

    _ros = make_unique<RosSupport>(ros::NodeHandle("Controller_manager"));
    _stats_publisher = _ros->advertise<riccardo_giacchino_master_thesis::Custom_torque>("controller_manager", 1);

    _imu = _model->getImu("pelvis");

    return true;

}

void ControllerManager::on_start()
{

    // Robot and Model update
    _robot->sense();
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    _model->update();

    // Memorize the initial joint stiffness and damping
    _robot->getStiffness(_stiff_initial_state);
    _robot->getDamping(_damp_initial_state);

    // Set joint stiffness and damping to zero in order to disable joint impedance controller
    if (!_robot->setStiffness(_stiff_tmp_state) || !_robot->setDamping(_damp_tmp_state))
        cout << "[ERROR]: unable to set stiffness or damping value" << endl;

    _robot->move();

    _time = 0;
    _solver->reset(_time);

    // activate ros wrapper
    _ros_wrapper->activate(true);

    cout << "[INFO]: Controller is starting!" << endl;
}

void ControllerManager::run()
{
    //
    _ros_wrapper->receive();

    _robot->sense();
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    //----------------
    _imu->getOrientation(_floating_base_orientation);
    _model->setFloatingBaseOrientation(_floating_base_orientation);
    _model->update();

    // Update the reference values for the
    if (stab_controller_enable == true){

        for (auto& pair : _stability_controller){

            _stability_controller[pair.first]->update(_time, _dt);

        }
    }

    // CartesIO pluing update will compute the torque base on the controller and set them into the model
    _solver->update(_time, _dt);

    // Get the torque from the model
    _model->getJointEffort(_torque_cartesian);

    compute_gravity_compensation();

    control_law();

    // Set the torque in the robot
    _robot->setEffortReference(_torque.tail(_robot->getJointNum()));

    _model->update();

    // keep updated the position reference
    _robot->getMotorPosition(_motor_position);
    _robot->setPositionReference(_motor_position);

    // Keep disabled joint impedance controller
    _robot->setStiffness(_stiff_tmp_state);
    _robot->setDamping(_damp_tmp_state);

    _robot->move();

    _stats_publisher->publish(_msg);

    //
    _ros_wrapper->send();

    // ============================== LOGGER ==============================
    // _imu->getOrientation(orient);
    // _logger->add("Roll_angle", atan2(orient(2, 1), orient(2, 2)));
    // for (auto task : _tasks_casted){
    //     _model->getPose(task->getDistalLink(), task->getBaseLink(), pos_real);
    //     task->getPoseReference(pos_ref);
    //     _logger->add(string("pos_real_" + task->getName()), pos_real.translation());
    //     _logger->add(string("pos_ref_" + task->getName()), pos_ref.translation());
    //     _logger->add(string("pos_err_" + task->getName()), pos_ref.translation() - pos_real.translation());
    // }
    // _logger->add("time", _time);


    _imu->getImuData(_floating_base_orientation,
                     _angular_vel_imu,
                     _linear_acc_imu);

    _msg.roll_angle = atan2(_floating_base_orientation(2, 1), _floating_base_orientation(2, 2));
    _msg.pitch_angle = atan2(-_floating_base_orientation(2, 0), sqrt(_floating_base_orientation(2, 2) * _floating_base_orientation(2, 2) + _floating_base_orientation(2, 1) * _floating_base_orientation(2, 1)));
    _msg.time = _time;
    _msg.angular_velocity.angular.x = _angular_vel_imu.x();
    _msg.angular_velocity.angular.y = _angular_vel_imu.y();
    _msg.angular_velocity.angular.z = _angular_vel_imu.z();
    _msg.linear_acceleration.linear.x = _linear_acc_imu.x();
    _msg.linear_acceleration.linear.y = _linear_acc_imu.y();
    _msg.linear_acceleration.linear.z = _linear_acc_imu.z();

    // Update the time for the solver
    _time += _dt;

}

void ControllerManager::on_stop()
{
    //
    _ros_wrapper->activate(false);

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    _model->update();

    // Reset the joint stiffness and damping to initial values in order to enable joint impedance controller
    _robot->setStiffness(_stiff_initial_state);
    _robot->setDamping(_damp_initial_state);

    _robot->move();

    cout << "[INFO]: Controller is stopping!" << endl;

    // ============================== DEBUG ==============================
    _logger.reset();

}

void ControllerManager::on_close()
{
    //
    _ros_wrapper->close();
}

// ==============================================================================
// Additional Functions
// ==============================================================================

void ControllerManager::get_task(){

    for (auto task : _tasks){

        // Cast from Generic task to Interaction task
        auto tmp_task = std::dynamic_pointer_cast<InteractionTask>(task);

        // Check null pointer
        if (tmp_task == nullptr)
            cerr << "[ERROR]: the cast result in a null pointer" << endl;
        else
            _tasks_casted.push_back(tmp_task);   // cast into Interaction task

    }

}

void ControllerManager::joint_map_generator(){

    auto urdf_model = _model->getUrdf();

    // Vector of which joint to set in Idle control mode
    auto vec = {"ankle_yaw_1", "ankle_yaw_2", "ankle_yaw_3", "ankle_yaw_4"};

    for (auto task : _tasks_casted) {

        string end_link = task->getDistalLink();

        while (end_link != task->getBaseLink() && end_link != "pelvis") {

            auto link = urdf_model.getLink(end_link);

            auto parent_joint = link->parent_joint;

            if (parent_joint == nullptr) {
                cerr << "[ERROR]: null pointer to parent joint on link " << end_link << endl;

            } else if (!_robot->hasJoint(parent_joint->name)) {
                cerr << "[WARNING]: robot does not have joint " << parent_joint->name << endl;

            } else {

                // Setting the yaw joint of each leg to Idle so that it is possible to use the omnisteering plugin
                if (find(vec.begin(), vec.end(), parent_joint->name) != vec.end()){
                    _ctrl_map[parent_joint->name] = ControlMode::Idle();
                }

                else{

                    _ctrl_map[parent_joint->name] = ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping() + ControlMode::Position();
                    _stiff_tmp_state[parent_joint->name] = _zero;
                    _damp_tmp_state[parent_joint->name] = _zero;

                }
            }

            end_link = link->getParent()->name;

        }

    }

}

void ControllerManager::compute_gravity_compensation(){

    _model->computeGravityCompensation(_gravity_torque);

    _g = _gravity_torque.head(6);   // isolate the torque related to the floating base, so the first 6 elements

    // cout << "Floating base gravity:\n" << _g << endl;

    // Computation of the contact force

    for (int i = 0; i < _tasks_casted.size(); i++) {

        string end_link = _tasks_casted[i]->getDistalLink();

        _model->getJacobian(end_link, _J_c[i]); // _J_c[i] is a 6 x 46 matrix

        // cout << end_link << "\n" << _J_c[i].block(0, 0, 6, 6) << endl;

        _J_cz.col(i).noalias() = _J_c[i].transpose().block(0, 2, 1, 6);

    }

    // cout << "J_fb:\n" << _J_cz << endl;
    // cout << ".............................." << endl;

    _J_cz_pseudo_inverse.noalias() = _J_cz.completeOrthogonalDecomposition().pseudoInverse();   // pseudo inverse computation

    // cout << "J_pseudo inverse:\n" << _J_cz_pseudo_inverse << endl;
    // cout << "--------------------------" << endl;

    _contact_force_z.noalias() = -(_J_cz_pseudo_inverse * _g);   // F_contact

    // cout << "contact force:\n "<< _contact_force_z.transpose() << endl;

    // τ = -τ_cartesian -τ_contact + g

    _torque_contact.setZero(_model->getJointNum());

    for (int i = 0; i < _tasks_casted.size(); i++){

        wrench.setZero();

        wrench[2] = _contact_force_z[i];

        // ------------- LOGGER -------------
        _msg.contact_forces[i] = _contact_force_z[i];

        _model->getJacobian(_tasks_casted[i]->getDistalLink(),
                            _J_leg[i]);

        _torque_contact.noalias() += _J_leg[i].transpose() * wrench;

    }

}

void ControllerManager::control_law(){

    current_index = 6;

    for (int i = 0; i < _tasks_casted.size(); i++){

        _model->computeRelativeJdotQdot(_tasks_casted[i]->getDistalLink(), _tasks_casted[i]->getBaseLink(), _J_dot_Q_dot);

        _total_Jd_Qd.segment(current_index, _J_dot_Q_dot.size()) = _J_dot_Q_dot;
        current_index += _J_dot_Q_dot.size();
    }

    _model->computeNonlinearTerm(_non_linear_torque);

    // τ = g_a + (C + J\dot) * q\dot + τ_cartesian
    _torque.noalias() = _torque_cartesian + _torque_contact + _non_linear_torque + _total_Jd_Qd;

    // cout << _torque.head(6).transpose() << endl;

    // ------------- LOGGER -------------
    // cout << _torque_contact.head(6).transpose() << endl;
    // _logger->add("non_linear_terms", _non_linear_torque);
    // _logger->add("gravity", _gravity_torque);
    // _logger->add("contact", _torque_contact);

    // ------------- LOGGER -------------
    vectorEigenToMsg();

}

void ControllerManager::stability_controller_initialization(){

    int i = 0;

    for (auto it = _config_parameters_stab_controller.begin(); it != _config_parameters_stab_controller.end(); ++it) {

        // Create StabilityController with the read parameters
        _stability_controller[_tasks_casted[i]] = std::make_unique<StabilityCompensation>(_model,
                                                                                          _tasks_casted[i],
                                                                                          it);
        i++;
    }

}

void ControllerManager::vectorEigenToMsg(){

    for (int i = 0; i < _model->getJointNum(); i++){

        _msg.J_dot_q_dot[i] = _total_Jd_Qd[i];
        _msg.non_linear_torque[i] = _non_linear_torque[i];
        _msg.contact_torque[i] = _torque_contact[i];
        _msg.torque[i] = _torque[i];

    }

}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)
