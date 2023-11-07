#ifndef CARTESIANIMPEDANCECONTROLLER_H
#define CARTESIANIMPEDANCECONTROLLER_H

// -------------------------- INCLUDE -------------------------- //

#include <iostream>
#include <thread>
#include <math.h>
#include <chrono>

#include <cartesian_interface/utils/RobotStatePublisher.h> // ROS related
#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace XBot;

// -------------------------- CLASS -------------------------- //

/**
 * @brief The CartesianImpedanceController class
 */
class CartesianImpedanceController
{

public:

    /**
     * @brief CartesianImpedanceController constructor
     */
    CartesianImpedanceController(ros::NodeHandle nh, double dt);

    /**
     * @brief CartesianImpedanceController destructor
     */
    ~CartesianImpedanceController();

    // Functions

    /**
     * @brief update_inertia is a function that update the operational space inertia matrix and compute the matrix Q
     * from its Cholesky factor
     */
    void update_inertia();

    /**
     * @brief update_K_and_D is a function the update the value of the stiffness and damping matrix due to the change
     * in the operational space inertia
     */
    void update_K_and_D();

    /**
     * @brief compute_force is the function that given all the error of acc, vel and pos compute the virtual force of
     * the damper and spring system, using the previously updated operational space inertia, stiffness and damping matrix
     * @return return the wrench containig the force and moment
     */
    Eigen::Vector6d compute_force();

    // Setter and Getter

    /**
     * @brief set_stiffness_damping
     * @param newK_diag
     * @param newD_diag
     */
    void set_stiffness_damping(const Eigen::Matrix6d &newK_diag, const Eigen::Matrix6d &newD_diag);

    /**
     * @brief get_stiffness
     * @return
     */
    Eigen::Matrix6d get_stiffness() const;

    /**
     * @brief get_damping
     * @return
     */
    Eigen::Matrix6d get_damping() const;

    /**
     * @brief set_reference_value is used to set the position and
     * @param acc_ref is the reference acceleration value of the end effector
     * @param vel_ref is the reference velocity value of the end effector
     * @param pos_ref is the reference position value of the end effector
     */
    void set_reference_value(Eigen::Vector6d acc_ref, Eigen::Vector6d vel_ref, Eigen::Vector6d pos_ref);

private:

    // Variables

    double _dt; // Sampling time

    const string _root_link;
    const string _end_effector_link;

    ros::NodeHandle _nh;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;

    Eigen::Matrix6d _K_diag, _D_diag;   // Diagonal matrix that represent the elementary stiffness and damping of the Cartesian axis
    Eigen::Matrix6d _K, _D; // Computed Stiffness and damping matrix
    Eigen::Matrix6d _op_sp_inertia; // Operational space inertial matrix, usually referred to as Λ

    // Reference acceleration, velocity, position of the end-effector w.r.t. to the base link
    Eigen::Vector6d _xddot_ref = Eigen::Vector6d::Zero();
    Eigen::Vector6d _xdot_ref = Eigen::Vector6d::Zero();
    Eigen::Vector6d _x_ref = Eigen::Vector6d::Zero();

    Eigen::Vector6d _xddot_real, _xdot_real, _x_real;  // Actual acceleration, velocity, position of the end-effector w.r.t. to the base link

    Eigen::Vector6d _eddot, _edot, _e; // Error between the actual and reference values

    Eigen::MatrixXd _J; // Jacobian matrix between the root link and the end effector
    Eigen::MatrixXd _B; // Inertia matrix in joint space
    Eigen::Matrix6d _Q; // Used in the Cholesky decomposition of the operational space inertia

    // Functions

    /**
     * @brief cholesky_decomp compute the Cholesky decomposition of the operational space inertia
     * in order to obtain the matrix Q used in the computation of stiffness (K) and damping (D)
     */
    void cholesky_decomp();

    /**
     * @brief matrix_sqrt compute the square root of each elements of the matrix
     * @param matrix
     * @return the square root of the input matrix
     */
    Eigen::Matrix6d matrix_sqrt(Eigen::Matrix6d matrix);

};

#endif // CARTESIANIMPEDANCECONTROLLER_H
