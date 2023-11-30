#include "controllermanager.h"

bool ControllerManager::on_initialize()
{

    _robot->sense();

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    _model->update();

    /* Read stiffness value from YAML file */
    vector<string> leg_chains = getParamOrThrow<vector<string>>("~chain_names");
    vector<string> _end_effector_links = getParamOrThrow<vector<string>>("~end_effector_links");
    vector<double> stiffness_front = getParamOrThrow<vector<double>>("~stiffness_front");
    vector<double> stiffness_back = getParamOrThrow<vector<double>>("~stiffness_back");
    double damping_factor = getParamOrThrow<double>("~damping");

    Eigen::Map<Eigen::Vector6d> map1(stiffness_front.data());
    Eigen::Map<Eigen::Vector6d> map2(stiffness_back.data());

    _stiffness.push_back(map1);
    _stiffness.push_back(map1);
    _stiffness.push_back(map2);
    _stiffness.push_back(map2);

    int i = 0;

    for (string chain : leg_chains){

        if (!_robot->hasChain(chain)){

            cout << "[ERROR]: robot does not have chain " << chain << endl;
            return false;

        } else{

            RobotChain& leg = _robot->chain(chain);

            _legs_controller.push_back(
                std::make_unique<CartesianImpedanceController>(_model,
                                                               _stiffness[i].asDiagonal(),
                                                               _end_effector_links[i],
                                                               leg.getBaseLinkName(),
                                                               damping_factor));
            i++;

            for (string joint_name : leg.getJointNames()){

                if (!_robot->hasJoint(joint_name)){
                    //cout << "[ERROR]: robot does not have joint " << joint_name << endl;
                    return false;
                } else {
                    joint_names.push_back(joint_name);
                    _ctrl_map[joint_name] = ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping();
                    _stiff_tmp_state[joint_name] = 0.0;
                    _damp_tmp_state[joint_name] = 0.0;
                }
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

    if (!_robot->setStiffness(_stiff_tmp_state) || !_robot->setDamping(_damp_tmp_state))
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

    _robot->setEffortReference(effort);

    _robot->setStiffness(_stiff_tmp_state);
    _robot->setDamping(_damp_tmp_state);

    _robot->move();

}

void ControllerManager::on_stop()
{

    _robot->setStiffness(_stiff_initial_state);
    _robot->setDamping(_damp_initial_state);

    _robot->move();


    for (auto& leg : _legs_controller){
        leg->reset_logger();
    }

    cout << "[INFO]: Cartesian impedance control is stopping!" << endl;
}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)
