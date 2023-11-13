#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H

#include <cartesianimpedancecontroller.h>

using namespace XBot;

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

    ModelInterface::Ptr _model;

    Eigen::VectorXd _stiff_initial_state, _effort_initial_state;

    std::vector<CartesianImpedanceController> _legs_controller;

    std::vector<string> _end_effector_link_names;

    Eigen::Vector6d _stiffness;

    int _n_joints = 0;
};

#endif // CONTROLLERMANAGER_H
