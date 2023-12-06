#include "cartesianimpedancesolver.h"

CartesianImpedanceSolver::CartesianImpedanceSolver(ProblemDescription ik_problem,
                                                   XBot::Cartesian::Context::Ptr context):
    CartesianInterfaceImpl(ik_problem, context)
{

    _tasks = ik_problem.getTask(ik_problem.getNumTasks()-1);

    for (auto task : _tasks){

        auto tmp_task = std::dynamic_pointer_cast<InteractionTask>(task);

        if (tmp_task == nullptr)
            cerr << "[ERROR]: the cast result in a null pointer" << endl;
        else
            _tasks_casted.push_back(tmp_task);   // cast into Interaction task

    }

    for (auto& task_casted : _tasks_casted){

        Impedance imp = task_casted->getImpedance();

        _controller[task_casted] = std::make_unique<CartesianImpedanceController>(_model,  // always updated
                                                                                  imp.stiffness,
                                                                                  imp.damping,
                                                                                  task_casted->getDistalLink(),
                                                                                  task_casted->getBaseLink());

    }

    // Variable Initialization
    _effort = Eigen::VectorXd::Zero(_model->getJointNum());

    _Tref = Eigen::Affine3d::Identity();

}

bool CartesianImpedanceSolver::update(double time, double period){

    _effort = Eigen::VectorXd::Zero(_model->getJointNum()); // reset to zero the effort

    for (auto& pair : _controller){

        pair.first->getImpedance();
        pair.first->getPoseReference(_Tref);

        _controller[pair.first]->set_stiffness(pair.first->getImpedance().stiffness);
        _controller[pair.first]->set_damping_factor(pair.first->getImpedance().damping);
        _controller[pair.first]->set_reference_value(_Tref);

        _effort += _controller[pair.first]->compute_torque();

    }

    _model->setJointEffort(_effort);

    return true;
}


CARTESIO_REGISTER_SOLVER_PLUGIN(CartesianImpedanceSolver, ImpSolver);
