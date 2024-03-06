#include "cartesianimpedancesolver.h"

// ==============================================================================
// Constructor
// ==============================================================================

CartesianImpedanceSolver::CartesianImpedanceSolver(ProblemDescription ik_problem,
                                                   XBot::Cartesian::Context::Ptr context):
    CartesianInterfaceImpl(ik_problem, context)
{

    // Extract tasks and cast to InteractionTask
    _tasks = ik_problem.getTask(ik_problem.getNumTasks()-1);

    for (auto task : _tasks){

        auto tmp_task = std::dynamic_pointer_cast<InteractionTask>(task);

        if (tmp_task == nullptr)
            cerr << "[ERROR]: the cast result in a null pointer" << endl;
        else
            _tasks_casted.push_back(tmp_task);   // cast into Interaction task

    }

    // Create all controller for each InteractionTask
    for (auto& task_casted : _tasks_casted){

        _imp = task_casted->getImpedance();

        _impedance_controller[task_casted] = std::make_unique<CartesianImpedanceController>(_model,  // always updated
                                                                                  _imp.stiffness,
                                                                                  _imp.damping,
                                                                                  task_casted->getDistalLink(),
                                                                                  task_casted->getBaseLink(),
                                                                                  task_casted->getName());

        // _imp.mass = _impedance_controller[task_casted]->get_Mass();
        // task_casted->setImpedance(_imp);

    }

    // Variable Initialization
    _effort = Eigen::VectorXd::Zero(_model->getJointNum());
    _Tref = Eigen::Affine3d::Identity();

}

// ==============================================================================
// Additional Functions
// ==============================================================================

bool CartesianImpedanceSolver::update(double time, double period){

    _effort.setZero();

    for (auto& pair : _impedance_controller){

        _imp = pair.first->getImpedance();
        pair.first->getPoseReference(_Tref, &_vel_ref, &_acc_ref);

        _impedance_controller[pair.first]->set_stiffness(_imp.stiffness);
        _impedance_controller[pair.first]->set_damping_factor(_imp.damping);

        _impedance_controller[pair.first]->set_reference_value(_Tref, _vel_ref, _acc_ref);

        _effort += _impedance_controller[pair.first]->compute_torque();

    }

    _model->setJointEffort(_effort);

    return true;
}

CARTESIO_REGISTER_SOLVER_PLUGIN(CartesianImpedanceSolver, ImpSolver);
