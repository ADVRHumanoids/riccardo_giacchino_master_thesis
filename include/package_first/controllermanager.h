#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H

#include <cartesianimpedancecontroller.h>
#include <cartesianimpedancesolver.h>
#include <urdf_model/model.h>

using namespace XBot::Cartesian;

class ControllerManager : public ControlPlugin
{
public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void on_start() override;

    void run() override;

    void on_stop() override;

private:

    std::map<std::string, ControlMode> _ctrl_map;

    double _zero = 0.0; // it s the value at which we want to set the stiffness
                        //and damping to deactivate the joint impedance controller

    XBot::ModelInterface::Ptr _model;
    Eigen::VectorXd _q, _qdot;

    JointNameMap _stiff_initial_state, _damp_initial_state;
    JointNameMap _stiff_tmp_state, _damp_tmp_state;
    JointNameMap _motor_position;

    vector<string> joint_names;

    shared_ptr<CartesianInterfaceImpl> _solver;
    shared_ptr<XBot::Cartesian::Context> _ctx;

    double _dt, _time;

    Eigen::VectorXd _torque;



    AggregatedTask _tasks;
    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;

    void joint_map_generator();

    void get_task_names();

    void notify_starting();

    void notify_ending();

};

#endif // CONTROLLERMANAGER_H
