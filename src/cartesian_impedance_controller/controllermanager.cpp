#include "controllermanager.h"

//NOTE: getParamOrThrow will take the the parameters from the YAML file, so they have to be defined in it

bool ControllerManager::on_initialize()
{

    _robot->sense();

    // TODO: about the derivation time, how it works

    // BUG: this function will always return TRUE

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // Setting the control mode of each control joint to Effort
    _robot->setControlMode(ControlMode::Idle());

    /* Read stiffness value from YAML file
    auto stiffness = getParamOrThrow<vector<double>>("~stiffness");


    if (stiffness.size() < 6){
        cout << "[ERROR]: stiffness size is less than 6" << endl;
        return false;
    }
    */
    _stiffness << 1, 1, 1, 1, 1, 1;

    auto leg_chains = getParamOrThrow<vector<string>>("~chain_names");

    for (string chain : leg_chains){

        // DEBUG: print the name of the chain
        cout << "[INFO]: " << chain << endl;

        if (!_robot->hasChain(chain)){

            cout << "[ERROR]: robot does not have chain " << chain << endl;
            return false;

        } else {

            RobotChain& leg = _robot->chain(chain);

            _legs_controller.push_back(
            CartesianImpedanceController(_model,
                                         leg.getTipLinkName(),
                                         leg.getBaseLinkName(),
                                          _stiffness.asDiagonal()));

            cout << "[INFO]: joints of chian " << chain << endl;

            for (string joint_name : leg.getJointNames()){

                //DEBUG: print the joint names of the respective chain
                cout << joint_name << endl;

                if (!_robot->hasJoint(joint_name)){

                    cout << "[ERROR]: robot does not have joint " << joint_name << endl;
                    return false;

                } else {

                    _ctrl_map[joint_name] = ControlMode::Effort();
                    _stiff_initial_state[joint_name] = _stiff_tmp_state[joint_name] = 0.0;
                    _effort_initial_state[joint_name] = _effort_tmp_state[joint_name] = 0.0;

                }

            }

        }

    }


//    for (int i = 1; i <= _end_effector_link_names.size(); i++){

//        RobotChain& leg = _robot->leg(i);

//        _legs_controller.push_back(
//            CartesianImpedanceController(_model,
//                                         leg.getTipLinkName(),
//                                         leg.getBaseLinkName(),
//                                          _stiffness.asDiagonal()));

//        for (string joint_name : leg.getJointNames()){
//            _ctrl_map[joint_name] = ControlMode::Effort();

//        }

//    }

    setDefaultControlMode(_ctrl_map);

    return true;

}

void ControllerManager::on_start()
{

    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    _robot->getStiffness(_stiff_initial_state);

    _robot->getEffortReference(_effort_initial_state);

    if (_stiff_initial_state.size() != _effort_initial_state.size())
        cout << "[ERROR]: different size" << endl;

    _robot->setStiffness(_stiff_tmp_state);
    _robot->setEffortReference(_effort_tmp_state);

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
