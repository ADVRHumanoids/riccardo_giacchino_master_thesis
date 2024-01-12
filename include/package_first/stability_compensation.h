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
                          std::shared_ptr<Cartesian::InteractionTask> task,
                          string relative_leg,
                          double K_v,
                          double K_p);

    void update(double time, double period);

private:

    ModelInterface::Ptr _model;

    std::shared_ptr<Cartesian::InteractionTask> _task;

    string _comparison_leg;

    ImuSensor::ConstPtr _imu;

    Eigen::Matrix3d _orientation_matrix;
    Eigen::Vector3d _angular_vel;
    Eigen::Vector3d _linear_acc;

    Eigen::Affine3d _leg_pose;
    Eigen::Affine3d _relative_leg_pose;
    Eigen::Affine3d _tmp;

    double _K_v, _K_p;

    double _roll_angle, _roll_vel;
    double _const_dist;

    double _vel;
    double _pos, _pos_ref;

    double _acc;

    double _pos_err;

    void compute_position_error();

    void compute_velocity_error(double dt);

    void control_law();


};

#endif // STABILITY_COMPENSATION_H
