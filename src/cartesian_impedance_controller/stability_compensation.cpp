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

    _orientation = Eigen::Matrix3d::Identity();

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

    // get_IMU_velocity();

    // _angle = _IMU_angular_velocity.norm() * period;

    // // Debug print
    // //cout << "Angle:\n" << _angle << endl;

    // asSkewSymmetric(_IMU_angular_velocity);

    // _rotation_matrix = _rotation_matrix * (Eigen::Matrix3d::Identity() + (sin(_angle)) * _skew_symmetric_matrix/_IMU_angular_velocity.norm() + (1 - cos(_angle))/pow(_IMU_angular_velocity.norm(),2) * _skew_symmetric_matrix * _skew_symmetric_matrix);

    // Debug print
    //cout << "Rotation matrix from IMU\n" << _rotation_matrix << endl;
    //cout << "Period:\n" << period << endl;

    // Convertion to Roll angle in rad and degree
    //double roll = atan2(_rotation_matrix(2, 1), _rotation_matrix(2, 2));
    //double roll_deg = roll * 180.0 / M_PI;
    //std::cout << "Roll: " << roll << " radians / " << roll_deg << " degrees" << std::endl;

    _imu->getOrientation(_orientation);

    _model->getPose("contact_1", "base_link", pose1);
    _model->getPose("contact_2", "base_link", pose2);

    position1 = _orientation * pose1.translation();
    position2 = _orientation * pose2.translation();

    cout << position1.z() - position2.z() << endl;



}





