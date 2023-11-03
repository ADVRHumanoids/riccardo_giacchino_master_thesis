#include "cartesianimpedancecontroller.h"

CartesianImpedanceController::CartesianImpedanceController(ros::NodeHandle nh, double dt):
    _nh(nh),
    _dt(dt)
{
    XBot::ConfigOptions xbot_cfg = XBot::ConfigOptionsFromParamServer(_nh);
    _model = XBot::ModelInterface::getModel(xbot_cfg);

    try
    {
        _robot = XBot::RobotInterface::getRobot(xbot_cfg);
        _model->syncFrom(*_robot);
        _robot->setControlMode(XBot::ControlMode::Position());  //TODO: check if the control mode Position is still correct
        _robot->setControlMode(
            {
                {"j_wheel_1", XBot::ControlMode::Velocity()},
                {"j_wheel_2", XBot::ControlMode::Velocity()},
                {"j_wheel_3", XBot::ControlMode::Velocity()},
                {"j_wheel_4", XBot::ControlMode::Velocity()}
            });
    }
    catch (const std::exception& e)
    {
        // Setting homing postion of the robot
        Eigen::VectorXd qhome;
        _model->getRobotState("home", qhome);
        _model->setJointPosition(qhome);
        _model->update();
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);
        cerr << "[ Error ]: " << e.what() << endl;
    }
}

CartesianImpedanceController::~CartesianImpedanceController()
{
    cout << "[ END ]: The program is terminating..." << endl;
}


