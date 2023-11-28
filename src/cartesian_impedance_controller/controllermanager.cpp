#include "controllermanager.h"

//NOTE: getParamOrThrow will take the the parameters from the YAML file, so they have to be defined in it

bool ControllerManager::on_initialize()
{

    _robot->sense();

    // TODO: about the derivation time, how it works

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    _model->update();

    /* Read stiffness value from YAML file
    auto stiffness = getParamOrThrow<vector<double>>("~stiffness");

    if (stiffness.size() < 6){
        cout << "[ERROR]: stiffness size is less than 6" << endl;
        return false;
    }
    */
    _stiffness << 2000, 2000, 1000, 2000, 2000, 2000;

    string arm_chain = "left_arm";

    if (!_robot->hasChain(arm_chain)){

        cout << "[ERROR]: robot does not have chain " << arm_chain << endl;
        return false;

    } else{

        RobotChain& arm = _robot->chain(arm_chain);

        _legs_controller.push_back(
            std::make_unique<CartesianImpedanceController>(_model,
                                                           _stiffness.asDiagonal(),
                                                           arm.getTipLinkName(),
                                                           arm.getBaseLinkName()));

        for (string joint_name : arm.getJointNames()){

            if (!_robot->hasJoint(joint_name)){

                //cout << "[ERROR]: robot does not have joint " << joint_name << endl;
                return false;

            } else {

                joint_names.push_back(joint_name);
                _ctrl_map[joint_name] = ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping();
                _stiff_tmp_state[joint_name] = 0.0;
                _damp_tmp_state[joint_name] = 0.0;    // Let's try to make it works just with the stiffness, leaving the joint damping set
            }

        }

    }


    setDefaultControlMode(_ctrl_map);

    return true;

}

void ControllerManager::on_start()
{

    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    _robot->getStiffness(_stiff_initial_state);
    _robot->getDamping(_damp_initial_state);

    cout << "[INFO]: Cartesian impedance control is starting!" << endl;

    if (!_robot->setStiffness(_stiff_tmp_state) /*|| !_robot->setDamping(_damp_tmp_state)*/)
        cout << "[ERROR]: unable to set stiffness or damping value" << endl;

    _robot->move();

    for (auto& leg : _legs_controller){
        leg->set_reference_value();
    }

}

void ControllerManager::run()
{

    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    Eigen::VectorXd effort = Eigen::VectorXd::Zero(40); // reset to zero the effort

    for (auto& leg : _legs_controller){

        effort += leg->compute_torque();

    }

    //cout << effort << endl;

    _robot->setEffortReference(effort);

    _robot->setStiffness(_stiff_tmp_state);
//    _robot->setDamping(_damp_tmp_state);

    _robot->move();

}

void ControllerManager::on_stop()
{

    _robot->setStiffness(_stiff_initial_state);
//    _robot->setDamping(_damp_initial_state);

    _robot->move();


    for (auto& leg : _legs_controller){
        leg->reset_logger();
    }

    cout << "[INFO]: Cartesian impedance control is stopping!" << endl;
}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)
