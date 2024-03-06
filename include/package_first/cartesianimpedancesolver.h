#ifndef CARTESIANIMPEDANCESOLVER_H
#define CARTESIANIMPEDANCESOLVER_H

// ==============================================================================
// Include
// ==============================================================================

#include <cartesianimpedancecontroller.h>
#include <stability_compensation.h>
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

// ==============================================================================
// Namespace
// ==============================================================================

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

    /**
     * @brief Constructs a Cartesian Impedance Solver instance using provided problem description and context.
     *
     * This constructor initializes a Cartesian Impedance Solver instance by extracting tasks from the provided
     * problem description. It casts tasks into InteractionTask objects. For each task, it retrieves impedance
     * values, creating a corresponding Cartesian Impedance Controller and associating it with the task.
     *
     * @param ik_problem The problem description used to initialize the solver.
     * @param context.
     */
    CartesianImpedanceSolver(ProblemDescription ik_problem, XBot::Cartesian::Context::Ptr context);

    // ==============================================================================
    // Additional functions
    // ==============================================================================

    /**
     * @brief Updates the Cartesian impedance solver based on controller information.
     *
     * This function updates the Cartesian impedance solver using controller information.
     * It resets the effort to zero, iterates through the controllers, retrieves impedance
     * and pose reference values, sets stiffness, damping, and reference values for each controller,
     * computes torque, and sets joint effort in the model.
     * Additionally, this function ensures that information regarding stiffness, damping, and
     * reference pose are updated in case of modifications made through specific topics.
     *
     * @return Boolean indicating the success of the update operation.
    */
    bool update(double time, double period) override;

private:

    // ==============================================================================
    // Variables
    // ==============================================================================

    AggregatedTask _tasks;  // vector of tasks obtained from the problem description

    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;    // tasks casted into Interaction Task

    // Map allows for the association of each InteractionTask pointer with a unique Cartesian Impedance Controller
    map<std::shared_ptr<InteractionTask>, std::unique_ptr<CartesianImpedanceController>> _impedance_controller;

    Eigen::VectorXd _effort;

    Eigen::Affine3d _Tref;  // to set the pose reference
    Eigen::Vector6d _vel_ref;
    Eigen::Vector6d _acc_ref;

    bool bol = false;

    Impedance _imp;

};

#endif // CARTESIANIMPEDANCESOLVER_H
