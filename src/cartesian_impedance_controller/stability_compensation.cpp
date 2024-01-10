#include "stability_compensation.h"

StabilityCompensation::StabilityCompensation(ModelInterface::Ptr model,
                                             std::shared_ptr<Cartesian::InteractionTask> task,
                                             string relative_leg,
                                             double K_v,
                                             double K_p):
    _model(model),
    _task(task),
    _comparison_leg(relative_leg),
    _K_p(K_p),
    _K_v(K_v)
{

    _imu = _model->getImu("pelvis");

    if (_imu == nullptr){
        cerr << "[ERROR]: Null pointer to the IMU" << endl;
    }

    _orientation_matrix = Eigen::Matrix3d::Identity();
    _leg_pose = _relative_leg_pose = _tmp = Eigen::Affine3d::Identity();

    _acc = _pos_err = _vel = _pos = 0;
    _roll_angle = 0;

}

void StabilityCompensation::compute_position_error(){

    _imu->getOrientation(_orientation_matrix);

    _roll_angle = atan2(_orientation_matrix(2, 1), _orientation_matrix(2, 2));

    // Debug print
    cout << "Roll angle: " << _roll_angle << endl;

}

void StabilityCompensation::control_law(){

    _roll_vel = - _K_p * (_roll_angle);

    // Debug print
    cout << "αdot: " << _roll_vel << endl;
}

void StabilityCompensation::compute_velocity_error(double dt){

    _model->getPose(_task->getDistalLink(), _comparison_leg, _tmp);
    _const_dist = _tmp.translation().z();
    _tmp = Eigen::Affine3d::Identity();

    _vel = abs(_const_dist) * cos(_roll_angle) * _roll_vel;

    _pos += dt * _vel;

    // Debug print
    cout << "Pos: " << _pos << endl;

}


void StabilityCompensation::update(double time, double period){

    compute_position_error();
    control_law();
    compute_velocity_error(period);

    _task->getPoseReference(_tmp);
    _tmp.translation().z() += _pos;
    _task->setPoseReference(_tmp);

    // FIXME: understand the strategy to select which leg has to be raised when the robot is tilting. The possible strategy are to move just a side of the robot or divide the motion
    //        into half on one side and half on the other side in a oppsite sign of the motion.

    // FIXME: check the correctness of the _pos sign with respect to the sign of the roll angle computed by the IMU sensor

}





