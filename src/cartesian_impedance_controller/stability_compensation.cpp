#include "stability_compensation.h"

StabilityCompensation::StabilityCompensation(ModelInterface::Ptr model,
                                             std::vector<std::shared_ptr<Cartesian::InteractionTask>>& tasks):
    _model(model),
    _tasks(tasks)
{

    _IMU_angular_velocity = Eigen::Vector3d::Zero();
    _rotation_matrix = Eigen::Matrix3d::Identity();

    _imu = _model->getImu("pelvis");

    if (_imu == nullptr){
        cerr << "[ERROR]: Null pointer to the IMU" << endl;
    }

    starting_position = vector<Eigen::Affine3d>(4, Eigen::Affine3d::Identity());

    //Setting the starting position for all the legs
    for (int i = 0; i < _tasks.size(); i++){

        _tasks[i]->getPoseReference(starting_position[i]);
    }

    _model->getPose("contact_1", "contact_2", pose);
    h = pose.translation().y();

    tmp = Eigen::Affine3d::Identity();

}

void StabilityCompensation::get_IMU_velocity(){

    _imu->getAngularVelocity(_IMU_angular_velocity);
    // Debug print
    //cout << "Angular velocity:\n" << _IMU_angular_velocity.x() << endl;

}

void StabilityCompensation::asSkewSymmetric(Eigen::Vector3d vector){

    _skew_symmetric_matrix <<  0, -vector(2), vector(1),
                               vector(2), 0, -vector(0),
                               -vector(1), vector(0), 0;

    //cout << "skew_matrix:\n" << _skew_symmetric_matrix << endl;

}

void StabilityCompensation::update(double time, double period){

    get_IMU_velocity();

    _angle = _IMU_angular_velocity.norm() * period;

    // Debug print
    //cout << "Angle:\n" << _angle << endl;

    asSkewSymmetric(_IMU_angular_velocity);

    _rotation_matrix = _rotation_matrix * (Eigen::Matrix3d::Identity() + (sin(_angle)) * _skew_symmetric_matrix/_IMU_angular_velocity.norm() + (1 - cos(_angle))/pow(_IMU_angular_velocity.norm(),2) * _skew_symmetric_matrix * _skew_symmetric_matrix);

    compute_RPY_angle();

    brain();

    // Debug print
    //cout << "Rotation matrix from IMU\n" << _rotation_matrix << endl;
    //cout << "Period:\n" << period << endl;

    // Convertion to Roll angle in rad and degree
    //double roll = atan2(_rotation_matrix(2, 1), _rotation_matrix(2, 2));
    //double roll_deg = roll * 180.0 / M_PI;
    //std::cout << "Roll: " << roll << " radians / " << roll_deg << " degrees" << std::endl;
}

void StabilityCompensation::compute_RPY_angle(){

    _roll = atan2(_rotation_matrix(2, 1), _rotation_matrix(2, 2));
    _pitch = std::atan2(-_rotation_matrix(2, 0), std::sqrt(_rotation_matrix(2, 1) * _rotation_matrix(2, 1) + _rotation_matrix(2, 2) * _rotation_matrix(2, 2)));

}

void StabilityCompensation::brain(){

    if(_roll < -0.05){

        //only the right legs have to be raised of about
        _model->getPose("contact_2", "base_link", tmp);
        tmp.translation().z() += abs(h*sin(_roll));
        _tasks[1]->setPoseReference(tmp);

        _model->getPose("contact_4", "base_link", tmp);
        tmp.translation().z() += abs(h*sin(_roll));
        _tasks[3]->setPoseReference(tmp);
    }

    if (_roll > 0.05){

        _model->getPose("contact_2", "base_link", tmp);
        tmp.translation().z() += -abs(h*sin(_roll));
        _tasks[1]->setPoseReference(tmp);

        _model->getPose("contact_4", "base_link", tmp);
        tmp.translation().z() += -abs(h*sin(_roll));
        _tasks[3]->setPoseReference(tmp);
    }

}




