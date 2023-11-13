#include "controllermanager.h"

bool ControllerManager::on_initialize()
{

    // TODO: set the stiffness of the controller for each leg
    // TODO: about the derivation time, how it works

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // TODO: set the name of the end effector and also find a way to set them in a better way
    _end_effector_link_names = {"", "", "", ""};
    _stiffness << 1,1,1,1,1,1;

    for (const string& link_name : _end_effector_link_names)
    {
        _legs_controller.push_back(CartesianImpedanceController(_model,link_name));
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

}

void ControllerManager::run()
{
    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);


}

void ControllerManager::on_stop()
{

}

