#include "stability_compensation.h"

// ==============================================================================
// Constructor and Destructor
// ==============================================================================

StabilityCompensation::StabilityCompensation(ModelInterface::Ptr model,
                                             std::shared_ptr<Cartesian::InteractionTask> task,
                                             string relative_leg_roll,
                                             string relative_leg_pitch,
                                             double K_p_roll,
                                             double K_p_pitch):
    _model(model),
    _task(task),
    _comparison_leg_roll(relative_leg_roll),
    _comparison_leg_pitch(relative_leg_pitch),
    _K_p_roll(K_p_roll),
    _K_p_pitch(K_p_pitch)
{

    // TODO: Convert all these variable into a vector of two elements

    _imu = _model->getImu("pelvis");

    if (_imu == nullptr){
        cerr << "[ERROR]: Null pointer to the IMU" << endl;
    }

    _K_v_roll = 0.3 * 2 * sqrt(_K_p_roll);
    _K_v_pitch = 0.3 * 2 * sqrt(_K_p_pitch);

    _orientation_matrix =  Eigen::Matrix3d::Identity();
    _angular_vel = _linear_acc = Eigen::Vector3d::Zero();

    _leg_pose = _relative_leg_pose = _reference_pose = Eigen::Affine3d::Identity();

    _reference_vel = _reference_acc = Eigen::Vector6d::Zero(6);

    _delta_z_ddot = _delta_z_dot = _delta_z = 0;
    _roll_angle = _roll_acc = 0;

    print_config_param();

    // ROS stuff
    _ros = make_unique<RosSupport>(ros::NodeHandle(string("stab_node_" + task->getName())));
    _stats_publisher = _ros->advertise<riccardo_giacchino_master_thesis::RollPitchController>(string("stab_controller_" + task->getName()), 1);
    _msg.name = task->getDistalLink();

}

void StabilityCompensation::compute_position_error(){

    _imu->getImuData(_orientation_matrix,
                     _linear_acc,
                     _angular_vel);

    _roll_angle = atan2(_orientation_matrix(2, 1), _orientation_matrix(2, 2));
    _pitch_angle = atan2(-_orientation_matrix(2, 0), sqrt(_orientation_matrix(2, 2) * _orientation_matrix(2, 2) + _orientation_matrix(2, 1) * _orientation_matrix(2, 1)));

    // rpy = _orientation_matrix.eulerAngles(0, 1, 2);
    // _roll_angle = rpy(0);
    // _pithc_angle = rpy(1);

    // ------------ SAFETY FEATURES ------------
    check_angle();

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

    _roll_acc = - _K_v_roll * (_angular_vel.x()) - _K_p_roll * (_roll_angle);

    _pitch_acc = - _K_v_pitch * (_angular_vel.y()) - _K_p_pitch * (_pitch_angle);

    // ------------ SAFETY FEATURES ------------
    check_computed_values();

    // ------------ DEBUG ------------
    // cout << "Roll action: " << _roll_acc << endl;
    // cout << "Pitch action: " << _pitch_acc << endl;

    // ------------ LOGGER ------------
    _msg.commanded_roll_acc = _roll_acc;
    _msg.commanded_pitch_acc = _pitch_acc;


}

void StabilityCompensation::convertion_to_leg_motion(double dt){

    // Getting the current position between the leg and its relative leg (roll)
    _model->getPose(_task->getDistalLink(), _comparison_leg_roll, _reference_pose);
    _const_dist_roll = -(_reference_pose.translation().y());    // extracting the distance along y direction

    // Getting the current position between the leg and its relative leg (pitch)
    _model->getPose(_task->getDistalLink(), _comparison_leg_pitch, _reference_pose_2);
    _const_dist_pitch = (_reference_pose_2.translation().x());  // extracting the distance along x direction

    // Resetting the values (not needed but for safety)
    _reference_pose = _reference_pose_2 = Eigen::Affine3d::Identity();

    _delta_z_ddot = _const_dist_roll * (- sin(_roll_angle) * pow(_angular_vel.x(), 2) + cos(_roll_angle) * _roll_acc * dt) +
                    _const_dist_pitch * (- sin(_pitch_angle) * pow(_angular_vel.y(), 2) + cos(_pitch_angle) * _pitch_acc * dt);

    _delta_z_dot = _const_dist_roll * cos(_roll_angle) * _roll_acc +
                   _const_dist_pitch * cos(_pitch_angle) * _pitch_acc;

    _delta_z = (dt * _delta_z_dot) + (0.5 * pow(dt, 2) * _delta_z_ddot);

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

}


void StabilityCompensation::update(double time, double period){

    compute_position_error();
    control_law();
    convertion_to_leg_motion(period);

    // Get reference values of the task
    _task->getPoseReference(_reference_pose, &_reference_vel, &_reference_acc);

    // ------------ LOGGER ------------
    tf::poseEigenToMsg(_reference_pose, _msg.old_reference_position);
    tf::twistEigenToMsg(_reference_vel, _msg.old_reference_velocity);
    tf::twistEigenToMsg(_reference_acc, _msg.old_reference_acceleration);

    // This check is redundant
    if (emergency_stop == false){

        // Update the reference values of the task with computed values
        _reference_pose.translation().z() += _delta_z/2;
        _reference_vel(2) = _delta_z_dot;
        _reference_acc(2) = _delta_z_ddot;

    } else {

        _reference_pose.translation().z() += 0.0;
        _reference_vel(2) = 0.0;
        _reference_acc(2) = 0.0;

    }

    // Set the updated values
    _task->setPoseReference(_reference_pose);
    _task->setVelocityReference(_reference_vel);
    _task->setAccelerationReference(_reference_acc);

    // ------------ LOGGER ------------
    tf::poseEigenToMsg(_reference_pose, _msg.reference_position);
    tf::twistEigenToMsg(_reference_vel, _msg.reference_velocity);
    tf::twistEigenToMsg(_reference_acc, _msg.reference_acceleration);

    _stats_publisher->publish(_msg);

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
    cout << "- Relative leg: " << _comparison_leg_roll << endl;
    cout << "PITCH CONTROLLER:" << endl;
    cout << "- K_p = " << _K_p_pitch << endl;
    cout << "- K_v = " << _K_v_pitch << endl;
    cout << "- Relative leg: " << _comparison_leg_pitch << endl;
    cout << "=====================================================" << endl;

}
