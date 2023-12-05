#include "cartesianimpedancesolver.h"

CartesianImpedanceSolver::CartesianImpedanceSolver(ProblemDescription ik_problem,
                                                   XBot::Cartesian::Context::Ptr context):
    CartesianInterfaceImpl(ik_problem, context)
{

    _tasks = ik_problem.getTask(ik_problem.getNumTasks());

    for (auto task : _tasks){

        auto tmp_task = std::dynamic_pointer_cast<InteractionTask>(task);

        if (tmp_task == nullptr)
            cerr << "[ERROR]: the cast result in a null pointer" << endl;
        else
            _tasks_casted.push_back(tmp_task);   // cast into Interaction task

    }

    for (auto task_casted : _tasks_casted){

        Impedance imp = task_casted->getImpedance();

        _legs_controller.push_back(
            std::make_unique<CartesianImpedanceController>(_model,  // I get this model from CartesianInterfaceImpl that is always kept updated
                                                           imp.stiffness,
                                                           imp.damping,
                                                           task_casted->getDistalLink(),
                                                           task_casted->getBaseLink()));

    }

    _effort = Eigen::VectorXd::Zero(46);

}

bool CartesianImpedanceSolver::update(double time, double period){

    _effort = Eigen::VectorXd::Zero(46); // reset to zero the effort

    for (auto& leg : _legs_controller){

        _effort += leg->compute_torque();

    }

    _model->setJointEffort(_effort);

    return true;
}
