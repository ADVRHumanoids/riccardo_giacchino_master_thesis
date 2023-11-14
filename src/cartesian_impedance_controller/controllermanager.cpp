#include "controllermanager.h"
#include <geometry_msgs/Wrench.h>

//NOTE: getParamOrThrow will take the the parameters from the YAML file, so they have to be defined in it

bool ControllerManager::on_initialize()
{

    _robot->sense();

    // TODO: about the derivation time, how it works

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
    _stiffness << 200, 200, 200, 1000, 1000, 0;

    auto leg_chains = getParamOrThrow<vector<string>>("~chain_names");

    for (string chain : leg_chains){

        if (!_robot->hasChain(chain)){

            cout << "[ERROR]: robot does not have chain " << chain << endl;
            return false;

        } else {

            RobotChain& leg = _robot->chain(chain);

            _legs_controller.push_back(
            CartesianImpedanceController(_model,
                                         leg,
                                          _stiffness.asDiagonal()));

            cout << "[INFO]: joints of chian " << chain << endl;

            for (string joint_name : leg.getJointNames()){

                if (!_robot->hasJoint(joint_name)){

                    cout << "[ERROR]: robot does not have joint " << joint_name << endl;
                    return false;

                } else {

                    joint_names.push_back(joint_name);
                    _ctrl_map[joint_name] = ControlMode::Stiffness();
                    _stiff_tmp_state[joint_name] = 1000.0;
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

    std::map<std::string, ControlMode> mp;
    _robot->getControlMode(mp);

//    for (const auto& pair : mp){
//        cout << pair.first << " - " << pair.second.isStiffnessEnabled() << endl;
//    }

//    JointNameMap mmaapp;

//    for (const auto& joint_name : joint_names) {
//        mmaapp[joint_name] = 60.0;
//    }

//    _robot->setEffortReference(mmaapp);

    _robot->getStiffness(_stiff_initial_state);

    cout << "[INFO]: Cartesian impedance control is starting!" << endl;

//    xbot_msgs::JointState msg;
//    msg.name = joint_names;
//    msg.stiffness = vector<float>(joint_names.size(), 0.0);

//    advertise("/xbotcore/joint_states", msg);

    vector<string> enabled_joint = _robot->getEnabledJointNames();
    cout << "provaaaaaa" << endl;
    for (const string& name : enabled_joint){
        cout << name << endl;
    }

    if (!_robot->setStiffness(_stiff_tmp_state))
        cout << "[ERROR]: unable to set stiffness value" << endl;

}

void ControllerManager::run()
{
    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    Eigen::VectorXd effort = Eigen::VectorXd::Zero(40);

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

    cout << "[INFO]: Cartesian impedance control is stopping!" << endl;
}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)
