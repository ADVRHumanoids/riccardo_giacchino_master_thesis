#ifndef STABILITY_COMPENSATION_H
#define STABILITY_COMPENSATION_H

// ==============================================================================
// Include
// ==============================================================================

#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <xbot2/ros/ros_support.h>
#include <riccardo_giacchino_master_thesis/RollPitchController.h>
#include <eigen_conversions/eigen_msg.h>

// ==============================================================================
// Namespace
// ==============================================================================

using namespace XBot;
using namespace std;

// ==============================================================================
// Class
// ==============================================================================

/**
 * @brief The StabilityCompensation class provides functionality for stability compensation in a robotic system.
 *
 * This class is designed to compute and apply stability compensation based on IMU data like angle of the base and angular velocity.
 */
class StabilityCompensation
{

public:

    // ==============================================================================
    // Constructor and Destructor
    // ==============================================================================

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
                          string relative_leg_roll,
                          string relative_leg_pitch,
                          double K_p_roll,
                          double K_p_pitch);

    // ==============================================================================
    // Additional Functions
    // ==============================================================================

    /**
     * @brief Update the stability compensation based on the current state of the system.
     *
     * @param time The current time.
     * @param period The time period between updates.
     */
    void update(double time, double period);

private:

    // ==============================================================================
    // Variables
    // ==============================================================================

    ModelInterface::Ptr _model;

    std::shared_ptr<Cartesian::InteractionTask> _task;

    string _comparison_leg_roll, _comparison_leg_pitch;

    ImuSensor::ConstPtr _imu;

    Eigen::Matrix3d _orientation_matrix;
    Eigen::Vector3d _angular_vel;
    Eigen::Vector3d _linear_acc;

    Eigen::Affine3d _leg_pose, _leg_pose_2;
    Eigen::Affine3d _relative_leg_pose, _relative_leg_pose_2;
    Eigen::Affine3d _reference_pose, _reference_pose_2;

    Eigen::Vector6d _reference_vel, _reference_acc;

    double _K_v_roll, _K_p_roll;
    double _K_v_pitch, _K_p_pitch;

    double _roll_angle, _roll_acc, _pitch_angle, _pitch_acc;
    double _const_dist_roll, _const_dist_pitch;

    double _delta_z_ddot;
    double _delta_z_dot;
    double _delta_z;

    // Ros support
    RosSupport::Ptr _ros;
    riccardo_giacchino_master_thesis::RollPitchController _msg;
    PublisherPtr<riccardo_giacchino_master_thesis::RollPitchController> _stats_publisher;

    // ==============================================================================
    // Additional Private Functions
    // ==============================================================================

    /**
     * @brief compute_position_error
     */
    void compute_position_error();

    /**
     * @brief convertion_to_leg_motion
     * @param dt
     */
    void convertion_to_leg_motion(double dt);

    /**
     * @brief control_law
     */
    void control_law();

    // ==============================================================================
    // Safety features
    // ==============================================================================

    bool emergency_stop = false;
    double _max_angle = 5 * M_PI / 180; // conversion from 15° to rad
    double _max_control_action = 1.0;    // TODO: Check the unit of this value
    /**
     * @brief check_angle
     */
    void check_angle();

    /**
     * @brief check_computed_values
     */
    void check_computed_values();

    /**
     * @brief print_IMU_data
     */
    void print_IMU_data();

    void print_config_param();


};

#endif // STABILITY_COMPENSATION_H
