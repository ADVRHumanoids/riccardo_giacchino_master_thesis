#ifndef CARTESIANIMPEDANCESOLVER_H
#define CARTESIANIMPEDANCESOLVER_H

#include <cartesianimpedancecontroller.h>
#include <iostream>
#include <thread>
#include <math.h>
#include <chrono>

#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <matlogger2/matlogger2.h>
#include <cartesian_interface/sdk/SolverPlugin.h>


using namespace Cartesian;
using namespace std;


// ==============================================================================
// Class
// ==============================================================================

class CartesianImpedanceSolver : public CartesianInterfaceImpl
{

public:

    // ==============================================================================
    // Constructor
    // ==============================================================================

    CartesianImpedanceSolver(ProblemDescription ik_problem, XBot::Cartesian::Context::Ptr context);

    // ==============================================================================
    // Additional functions
    // ==============================================================================

    bool update(double time, double period) override;

private:

    // ==============================================================================
    // Variables
    // ==============================================================================

    std::vector<std::unique_ptr<CartesianImpedanceController>> _legs_controller;

    AggregatedTask _tasks;

    map<std::shared_ptr<InteractionTask>, std::unique_ptr<CartesianImpedanceController>> _controller;

    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;

    Eigen::VectorXd _effort;

    Eigen::Affine3d _Tref;


};

#endif // CARTESIANIMPEDANCESOLVER_H
