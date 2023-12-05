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

    XBot::ModelInterface::Ptr _model;

    JointNameMap _stiff_initial_state, _damp_initial_state;
    JointNameMap _stiff_tmp_state, _damp_tmp_state;

    vector<string> joint_names;

    shared_ptr<CartesianImpedanceSolver> _solver;
    shared_ptr<XBot::Cartesian::Context> _ctx;
    double _dt;
    AggregatedTask _tasks;
    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;

};

#endif // CONTROLLERMANAGER_H
