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

    // Setter and Getter

    void set_K_and_D(const Eigen::Matrix6d &newK_diag, const Eigen::Matrix6d &newD_diag);

    Eigen::Matrix6d get_K_diag() const;

    Eigen::Matrix6d get_D_diag() const;

private:

    double _dt; //Sampling time

    ros::NodeHandle _nh;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;

    Eigen::Matrix6d _K_diag, _D_diag;   // Diagonal matrix that represent the elementary stiffness and damping of the Cartesian axis
    Eigen::Matrix6d _K, _D; // Computed Stiffness and damping matrix

    Eigen::Vector6d _xddot_ref, _xdot_ref, _x_ref; // Reference acceleration, velocity, position of the end-effector w.r.t. to the base link
    Eigen::Vector6d _xddot_real, _xdot_real, _x_real;  // Actual acceleration, velocity, position of the end-effector w.r.t. to the base link
    Eigen::Vector6d _eddot, _edot, _e; // Error between the actual and reference values



};

#endif // CARTESIANIMPEDANCECONTROLLER_H
