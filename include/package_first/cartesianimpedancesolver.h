#ifndef CARTESIANIMPEDANCESOLVER_H
#define CARTESIANIMPEDANCESOLVER_H

#include <cartesianimpedancecontroller.h>

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

    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;

    Eigen::VectorXd _effort;


};

#endif // CARTESIANIMPEDANCESOLVER_H
