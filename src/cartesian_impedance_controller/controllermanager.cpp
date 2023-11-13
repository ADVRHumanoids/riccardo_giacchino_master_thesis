#include "controllermanager.h"

bool ControllerManager::on_initialize()
{

    // TODO: about the derivation time, how it works

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // TODO: set the name of the end effector and also find a way to set them in a better way
    _end_effector_link_names = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    _stiffness << 1,1,1,1,1,1;

    for (const string& link_name : _end_effector_link_names)
    {
        _legs_controller.push_back(
            CartesianImpedanceController(_model,
                                         link_name,
                                         "pelvis",
                                         _stiffness.asDiagonal()));
    }

    // Setting the control mode of each control joint to Effort
    _robot->setControlMode(ControlMode::Idle());

    auto control_joints = getParamOrThrow<std::vector<std::string>>("~control_joints");

    jinfo("will control joints {} \n",
          fmt::join(control_joints, ", "));

    for(auto j : control_joints)
    {
        if(!_robot->hasJoint(j))
        {
            jerror("invalid joint '{}' \n", j);
            return false;
        }

        _ctrl_map[j] = ControlMode::Effort();
    }

    setDefaultControlMode(_ctrl_map);

    return true;

}

void ControllerManager::on_start()
{

    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    _robot->getStiffness(_stiff_initial_state);
    _robot->getEffortReference(_effort_initial_state);

    _n_joints = _stiff_initial_state.size();

    _robot->setStiffness(Eigen::VectorXd::Zero(_n_joints));
    _robot->setEffortReference(Eigen::VectorXd::Zero(_n_joints));

}

void ControllerManager::run()
{
    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    Eigen::VectorXd effort = Eigen::VectorXd::Zero(_n_joints);

    for (auto leg : _legs_controller){

        leg.update_model(_model);
        effort += leg.compute_torque();
    }

    _robot->setEffortReference(effort);

    _robot->move();

}

void ControllerManager::on_stop()
{

    _robot->setStiffness(_stiff_initial_state);
    _robot->setEffortReference(_effort_initial_state);

}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)
