#ifndef STABILITY_COMPENSATION_H
#define STABILITY_COMPENSATION_H



#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace XBot;
using namespace std;

class StabilityCompensation
{

public:

    StabilityCompensation(ModelInterface::Ptr model);

    void update(double time, double period);

private:

    ModelInterface::Ptr _model;

    Eigen::Vector3d _IMU_angular_velocity;

    double _angle;

    Eigen::Vector3d _RPY_angle;

    Eigen::Matrix3d _rotation_matrix;

    ImuSensor::ConstPtr _imu;

    Eigen::Matrix3d _skew_symmetric_matrix;

    void asSkewSymmetric(Eigen::Vector3d vector);

    void get_IMU_velocity();

    void compute_rotation_matrix();

    void compute_RPY_angle();

};

#endif // STABILITY_COMPENSATION_H
