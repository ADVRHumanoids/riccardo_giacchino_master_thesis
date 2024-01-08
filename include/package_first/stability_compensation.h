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

    StabilityCompensation(ModelInterface::Ptr model,
                          std::vector<std::shared_ptr<Cartesian::InteractionTask>>& tasks);

    void update(double time, double period);

private:

    ModelInterface::Ptr _model;
    std::vector<std::shared_ptr<Cartesian::InteractionTask>> _tasks;

    Eigen::Vector3d _IMU_angular_velocity;

    vector<Eigen::Affine3d> starting_position;

    Eigen::Affine3d pose, tmp;

    double h;
    double _angle;
    double _roll, _pitch;

    Eigen::Matrix3d _rotation_matrix;

    ImuSensor::ConstPtr _imu;

    Eigen::Matrix3d _skew_symmetric_matrix;

    void asSkewSymmetric(Eigen::Vector3d vector);

    void get_IMU_velocity();

    void compute_rotation_matrix();

    void compute_RPY_angle();

    void brain();

};

#endif // STABILITY_COMPENSATION_H
