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



private:

    ros::NodeHandle _nh;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    double _dt; //Sampling time
};

#endif // CARTESIANIMPEDANCECONTROLLER_H
