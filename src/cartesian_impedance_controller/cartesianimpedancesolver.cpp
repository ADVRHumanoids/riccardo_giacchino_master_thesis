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

    vector<string> vet = {"contact_2", "contact_1", "contact_4", "contact_3"};
    vector<string> vet2 = {"contact_3", "contact_4", "contact_1", "contact_2"};
    int i = 0;

    // Create all controller for each InteractionTask
    for (auto& task_casted : _tasks_casted){

        Impedance imp = task_casted->getImpedance();

        _impedance_controller[task_casted] = std::make_unique<CartesianImpedanceController>(_model,  // always updated
                                                                                  imp.stiffness,
                                                                                  imp.damping,
                                                                                  task_casted->getDistalLink(),
                                                                                  task_casted->getBaseLink(),
                                                                                  task_casted->getName());

        _stability_controller[task_casted] = std::make_unique<StabilityCompensation>(_model,
                                                                                     task_casted,
                                                                                     vet[i],
                                                                                     vet2[i]);

        i++;

    }

    // Variable Initialization
    _effort = Eigen::VectorXd::Zero(_model->getJointNum());
    _Tref = Eigen::Affine3d::Identity();

}

// ==============================================================================
// Additional Functions
// ==============================================================================

bool CartesianImpedanceSolver::update(double time, double period){

    _effort = Eigen::VectorXd::Zero(_model->getJointNum()); // reset to zero the effort

    for (auto& pair : _impedance_controller){

        pair.first->getImpedance();

        // Roll and pitch angle controller
        _stability_controller[pair.first]->set_K_p(pair.first->getImpedance().stiffness.diagonal()(2)/pair.second->get_K_omega_z());
        _stability_controller[pair.first]->update(time, period);

        // Cartesian controller stuff

        pair.first->getPoseReference(_Tref, &_vel_ref, &_acc_ref);

        _impedance_controller[pair.first]->set_stiffness(pair.first->getImpedance().stiffness);
        _impedance_controller[pair.first]->set_damping_factor(pair.first->getImpedance().damping);
        _impedance_controller[pair.first]->set_reference_value(_Tref, _vel_ref, _acc_ref);

        _effort += _impedance_controller[pair.first]->compute_torque();

    }

    _model->setJointEffort(_effort);

    return true;
}

CARTESIO_REGISTER_SOLVER_PLUGIN(CartesianImpedanceSolver, ImpSolver);
