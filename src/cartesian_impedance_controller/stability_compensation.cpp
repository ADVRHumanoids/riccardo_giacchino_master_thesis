#include "stability_compensation.h"

StabilityCompensation::StabilityCompensation(ModelInterface::Ptr model,
                                             std::shared_ptr<Cartesian::InteractionTask> task,
                                             string relative_leg,
                                             double K_p):
    _model(model),
    _task(task),
    _comparison_leg(relative_leg),
    _K_p(K_p)
{

    _imu = _model->getImu("pelvis");

    if (_imu == nullptr){
        cerr << "[ERROR]: Null pointer to the IMU" << endl;
    }

    _K_v = 2 * sqrt(_K_p);

    _orientation_matrix =  Eigen::Matrix3d::Identity();
    _angular_vel = _linear_acc = Eigen::Vector3d::Zero();

    _leg_pose = _relative_leg_pose = _reference_pose = Eigen::Affine3d::Identity();

    _reference_vel = _reference_acc = Eigen::Vector6d::Zero(6);

    _delta_z_ddot = _delta_z_dot = _delta_z = 0;
    _roll_angle = _roll_acc = 0;

}

void StabilityCompensation::compute_position_error(){

    _imu->getImuData(_orientation_matrix,
                     _linear_acc,
                     _angular_vel);

    _roll_angle = atan2(_orientation_matrix(2, 1), _orientation_matrix(2, 2));

}

void StabilityCompensation::control_law(){

    _roll_acc = - _K_v * (_angular_vel.x()) - _K_p * (_roll_angle);

}

void StabilityCompensation::compute_velocity_error(double dt){

    _model->getPose(_task->getDistalLink(), _comparison_leg, _reference_pose);
    _const_dist = -(_reference_pose.translation().y());

    _reference_pose = Eigen::Affine3d::Identity();

    _delta_z_ddot = _const_dist * (- sin(_roll_angle) * pow(_angular_vel.x(), 2) + cos(_roll_angle) * _roll_acc);

    _delta_z_dot = _const_dist * cos(_roll_angle) * _roll_acc;

    _delta_z = (dt * _delta_z_dot) + (0.5 * pow(dt, 2) * _delta_z_ddot);

}


void StabilityCompensation::update(double time, double period){

    compute_position_error();
    control_law();
    compute_velocity_error(period);

    _task->getPoseReference(_reference_pose, &_reference_vel, &_reference_acc);

    _reference_pose.translation().z() += _delta_z/2;
    _reference_vel(2) = _delta_z_dot;
    _reference_acc(2) = _delta_z_ddot;

    _task->setPoseReference(_reference_pose);
    _task->setVelocityReference(_reference_vel);
    _task->setAccelerationReference(_reference_acc);

}





