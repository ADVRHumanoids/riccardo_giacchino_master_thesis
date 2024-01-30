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

/**
 * @brief The StabilityCompensation class provides functionality for stability compensation in a robotic system.
 *
 * This class is designed to compute and apply stability compensation based on IMU data like angle of the base and angular velocity.
 */
class StabilityCompensation
{

public:

    /**
     * @brief Constructor for StabilityCompensation.
     *
     * @param model A pointer to the robot model interface.
     * @param task A shared pointer to the Cartesian interaction task (already casted into Interaction task) The following code will
     *             change the reference position of such task
     * @param relative_leg The name of the link to identify the leg with respect to is computed the angle (See README file for more details).
     * @param K_v The velocity gain for the control law.
     * @param K_p The proportional gain for the control law.
     */
    StabilityCompensation(ModelInterface::Ptr model,
                          std::shared_ptr<Cartesian::InteractionTask> task,
                          string relative_leg,
                          double K_p);

    /**
     * @brief Update the stability compensation based on the current state of the system.
     *
     * @param time The current time.
     * @param period The time period between updates.
     */
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
    Eigen::Affine3d _reference_pose;

    Eigen::Vector6d _reference_vel, _reference_acc;

    double _K_v, _K_p;

    double _roll_angle, _roll_acc;
    double _const_dist;

    double _delta_z_ddot;
    double _delta_z_dot;
    double _delta_z;

    void compute_position_error();

    void compute_velocity_error(double dt);

    void control_law();

};

#endif // STABILITY_COMPENSATION_H
