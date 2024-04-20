#include "stability_compensation.h"

// ==============================================================================
// Constructor and Destructor
// ==============================================================================

StabilityCompensation::StabilityCompensation(ModelInterface::Ptr model,
                                             std::shared_ptr<Cartesian::InteractionTask> task,
                                             YAML::const_iterator it):
    _model(model),
    _task(task)
{

    extract_data_from_YAML_file(it);

    _imu = _model->getImu("pelvis");

    if (_imu == nullptr){
        cerr << "[ERROR]: Null pointer to the IMU" << endl;
    }

    // _imu->getImuData(_orientation_matrix,
    //                  _linear_acc,
    //                  _angular_vel);

    // _roll_angle_ref = atan2(_orientation_matrix(2, 1), _orientation_matrix(2, 2));

    _K_v_roll = _damping_factor_roll * 2 * sqrt(_K_p_roll);
    _K_v_pitch = _damping_factor_pitch * 2 * sqrt(_K_p_pitch);

    _orientation_matrix =  Eigen::Matrix3d::Identity();
    _angular_vel = _linear_acc = Eigen::Vector3d::Zero();

    _leg_pose = _relative_leg_pose = _reference_pose = Eigen::Affine3d::Identity();

    _reference_vel = _reference_acc = Eigen::Vector6d::Zero(6);

    _task->getPoseReference(_reference_pose);
    _initial_ref = _reference_pose.translation().z();

    _delta_z_ddot = _delta_z_dot = _delta_z = 0;
    _roll_angle = _roll_acc = 0;
    _pitch_angle = _pitch_acc = 0;

    // _roll_vel = _pitch_vel = 0;

    roll_cmd = roll_dot_cmd = pitch_cmd = pitch_dot_cmd = 0;

    print_config_param();

    // ROS stuff
    _ros = make_unique<RosSupport>(ros::NodeHandle(task->getName()));
    _stats_publisher = _ros->advertise<riccardo_giacchino_master_thesis::RollPitchController>(string("stability_controller"), 1);
    _msg.name = task->getDistalLink();

    // Getting the current position between the leg and its relative leg (roll)
    _model->getPose(_task->getDistalLink(), _comparison_leg_roll, _reference_pose);
    _const_dist_roll = - (_reference_pose.translation().y());    // extracting the distance along y direction

    // Getting the current position between the leg and its relative leg (pitch)
    _model->getPose(_task->getDistalLink(), _comparison_leg_pitch, _reference_pose_2);
    _const_dist_pitch = (_reference_pose_2.translation().x());  // extracting the distance along x directions

}

void StabilityCompensation::compute_position_error(){

    _imu->getImuData(_orientation_matrix,
                     _linear_acc,
                     _angular_vel);

    _roll_angle = atan2(_orientation_matrix(2, 1), _orientation_matrix(2, 2));
    _pitch_angle = atan2(-_orientation_matrix(2, 0), sqrt(_orientation_matrix(2, 2) * _orientation_matrix(2, 2) + _orientation_matrix(2, 1) * _orientation_matrix(2, 1)));

    // ------------ SAFETY FEATURES ------------
    // check_angle();

    // ------------ DEBUG ------------
    // print_IMU_data();

    // ------------ LOGGER ------------
    _msg.roll_angle = _roll_angle;
    _msg.pitch_angle = _pitch_angle;
    _msg.angular_vel.x = _angular_vel.x();
    _msg.angular_vel.y = _angular_vel.y();
    _msg.angular_vel.z = _angular_vel.z();
}

// ==============================================================================
// Additional Functions
// ==============================================================================

void StabilityCompensation::control_law(){

    // // First order
    // _roll_vel = _K_p_roll * ( - _roll_angle);
    // //_pitch_vel = - _K_p_pitch * (_pitch_angle);

    // _roll_acc = _pitch_acc = 0;

    // Second order
    _roll_acc = - _K_v_roll * (_angular_vel.x()) - _K_p_roll * (_roll_angle);
    _pitch_acc = - _K_v_pitch * (_angular_vel.y()) - _K_p_pitch * (_pitch_angle);

    // ----------- SAFETY FEATURES -----------
    // check_computed_values();

    // ---------------- DEBUG ----------------
    // cout << "Roll action: " << _roll_acc << endl;
    // cout << "Pitch action: " << _pitch_acc << endl;

    // ---------------- LOGGER ----------------
    _msg.commanded_roll_acc = _roll_acc;
    _msg.commanded_pitch_acc = _roll_acc;


}

void StabilityCompensation::convertion_to_leg_motion(double dt){

    // Resetting the values (not needed but for safety)
    _reference_pose = _reference_pose_2 = Eigen::Affine3d::Identity();

    roll_dot_cmd += _roll_acc * dt;
    roll_cmd += roll_dot_cmd * dt + 0.5 * dt * dt * _roll_acc;

    pitch_dot_cmd += _pitch_acc * dt;
    pitch_cmd += pitch_dot_cmd * dt + 0.5 * dt * dt * _pitch_acc;

    _delta_z_ddot = (_const_dist_roll * (- sin(roll_cmd) * pow(roll_dot_cmd, 2) + cos(roll_cmd) * _roll_acc) +
                     _const_dist_pitch * (- sin(pitch_cmd) * pow(pitch_dot_cmd, 2) + cos(pitch_cmd) * _pitch_acc)) * 0.5;

    _delta_z_dot += _delta_z_ddot * dt;

    _delta_z = (dt * _delta_z_dot) + (0.5 * dt * dt * _delta_z_ddot);

    // _delta_z = (_const_dist_roll * cos(roll_cmd) * _roll_vel +
    //             _const_dist_pitch + cos(pitch_cmd) * _pitch_vel) * 0.5 * dt;

    //_delta_z = (_const_dist_roll * sin(_roll_angle) + _const_dist_pitch * sin(_pitch_angle)) * 0.5;

    // ------------ DEBUG ------------
    // cout << "Delta acceleration: " << _delta_z_ddot << endl;
    // cout << "Delta velocity: " << _delta_z_dot << endl;
    // cout << "Delta position: " << _delta_z << endl;

    // ------------ LOGGER ------------
    _msg.const_distance_roll = _const_dist_roll;
    _msg.const_distance_pitch = _const_dist_pitch;
    _msg.delta_pos = _delta_z;
    _msg.delta_vel = _delta_z_dot;
    _msg.delta_acc = _delta_z_ddot;
    _msg.commanded_roll_vel = roll_dot_cmd;
    _msg.commanded_roll_ang = roll_cmd;
    _msg.commanded_pitch_vel = pitch_dot_cmd;
    _msg.commanded_pitch_ang = pitch_cmd;

}


void StabilityCompensation::update(double time, double period){

    //compute_gain();   // automatically compute dynamic gain in order to control settling time of the step responce w.r.t. the cartesian imp controller
    compute_position_error();
    control_law();
    convertion_to_leg_motion(period);

    // Get reference values of the task
    _task->getPoseReference(_reference_pose, &_reference_vel, &_reference_acc);

    // ------------ LOGGER ------------
    tf::poseEigenToMsg(_reference_pose, _msg.old_reference_position);
    tf::twistEigenToMsg(_reference_vel, _msg.old_reference_velocity);
    tf::twistEigenToMsg(_reference_acc, _msg.old_reference_acceleration);

    // Update the reference values of the task with computed values
    _reference_pose.translation().z() += _delta_z;
    _reference_vel(2) = _delta_z_dot;
    _reference_acc(2) = _delta_z_ddot;

    // Set the updated values
    _task->setPoseReference(_reference_pose);
    _task->setVelocityReference(_reference_vel);
    _task->setAccelerationReference(_reference_acc);

    // ------------ LOGGER ------------
    tf::poseEigenToMsg(_reference_pose, _msg.reference_position);
    tf::twistEigenToMsg(_reference_vel, _msg.reference_velocity);
    tf::twistEigenToMsg(_reference_acc, _msg.reference_acceleration);

    _stats_publisher->publish(_msg);    // publish the message

}

// ==============================================================================
// Safety features
// ==============================================================================

void StabilityCompensation::check_angle(){

    // Check if the roll angle exceed limit values
    if (_roll_angle < -_max_angle || _roll_angle > _max_angle){
        _roll_angle = 0.0;
        emergency_stop = true;
        cerr << "[WARNING]: The roll angle is over the limit. Set to zero for safety reason" << endl;
    }

    // Check if the pitch angle exceed limit values
    if (_pitch_angle < -_max_angle || _pitch_angle > _max_angle){
        _pitch_angle = 0.0;
        emergency_stop = true;
        cerr << "[WARNING]: The pitch angle is over the limit. Set to zero for safety reason" << endl;
    }

}

void StabilityCompensation::check_computed_values(){

    if (abs(_roll_acc) > _max_control_action || abs(_pitch_acc) > _max_control_action){

        _roll_acc = _pitch_acc = 0.0;
        emergency_stop = true;
        cerr << "[WARNING]: Control action is over the ceiling BROOOOO!!! I'm gonna stop it" << endl;

    }

}

void StabilityCompensation::extract_data_from_YAML_file(YAML::const_iterator it){

    _comparison_leg_roll = it->second["Roll"]["relative_leg"].as<std::string>();
    _comparison_leg_pitch = it->second["Pitch"]["relative_leg"].as<std::string>();
    _K_p_roll = it->second["Roll"]["gain"].as<double>();
    _K_p_pitch = it->second["Pitch"]["gain"].as<double>();
    _damping_factor_roll = it->second["Roll"]["damping_factor"].as<double>();
    _damping_factor_pitch = it->second["Pitch"]["damping_factor"].as<double>();
    _max_angle = it->second["Safety_limit"]["max_angle"].as<int>() * M_PI / 180;
    _max_control_action = it->second["Safety_limit"]["max_control_action"].as<double>();
    _settling_time_factor = it->second["SettlingTime"].as<double>();

}

void StabilityCompensation::print_IMU_data(){

    cout << "IMU_info:" << endl;
    cout << "Roll angle: " << _roll_angle << endl;
    cout << "Pitch angle: " << _pitch_angle << endl;
    cout << "Angular velocity: " << _angular_vel.transpose() << endl;
    cout << "Linear acceleration: " << _linear_acc.transpose() << endl;

}

void StabilityCompensation::print_config_param(){

    cout << "[OK] Successfully created controller for task " << _task->getName() << endl;
    cout << "ROLL CONTROLLER:" << endl;
    cout << "- K_p = " << _K_p_roll << endl;
    cout << "- K_v = " << _K_v_roll << endl;
    cout << "- ζ = " << _damping_factor_roll << endl;
    cout << "- Relative leg: " << _comparison_leg_roll << endl;
    cout << "PITCH CONTROLLER:" << endl;
    cout << "- K_p = " << _K_p_pitch << endl;
    cout << "- K_v = " << _K_v_pitch << endl;
    cout << "- ζ = " << _damping_factor_pitch << endl;
    cout << "- Relative leg: " << _comparison_leg_pitch << endl;
    cout << "Settling time factor: " << _settling_time_factor << endl;
    cout << "=====================================================" << endl;

}

void StabilityCompensation::compute_gain(){

    cout << "Ti sto usando per cose losche" << endl;

    _K_p_pitch = _K_p_roll = (_task->getImpedance().stiffness(2,2) / _task->getImpedance().mass(2,2)) / pow(_settling_time_factor * 1.66 * _damping_factor_roll, 2);

    _K_v_roll = 2 * _damping_factor_roll * sqrt(_K_p_roll);
    _K_v_pitch = 2 * _damping_factor_pitch * sqrt(_K_p_pitch);

    cout << _task->getName() << endl;
    cout << "- K_p = " << _K_p_roll << endl;
    cout << "- K_v = " << _K_v_roll << endl;
    cout << "- ζ = " << _damping_factor_roll << endl;
}


