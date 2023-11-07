#include "cartesianimpedancecontroller.h"

// ---------------------------------------- CONSTRUCTOR ---------------------------------------- //

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

    _op_sp_inertia = Eigen::Matrix6d::Identity();
}

// ---------------------------------------- DESTRUCTOR ---------------------------------------- //

CartesianImpedanceController::~CartesianImpedanceController()
{
    cout << "[ END ]: The program is terminating..." << endl;
}


// ---------------------------------------- FUNCTIONS ---------------------------------------- //

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{
    return matrix.array().sqrt();
}


void CartesianImpedanceController::cholesky_decomp()
{

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(_op_sp_inertia);
    if (eigensolver.info() != Eigen::Success) {
        // TODO: handle eigenvalue decomposition failure
    }

    // Get the eigenvectors in Q and eigenvalues in Lambda
    _Q = eigensolver.eigenvectors();
    //_lambda = eigensolver.eigenvalues().real();

}

void CartesianImpedanceController::update_inertia()
{

    // TODO: understand how to compute the joint inertia just for a leg

    //_model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    //_model->getInertiaInverse(_B);

    _op_sp_inertia = Eigen::Matrix6d::Identity();

    cholesky_decomp();  // Store the resulting matrix in the variable _Q;

}

void CartesianImpedanceController::update_K_and_D()
{

    _K = _Q * _K_diag * _Q.transpose();

    _D = 2 * _Q * _D_diag * matrix_sqrt(_K_diag) * _Q.transpose();

}



Eigen::Vector6d CartesianImpedanceController::compute_force()
{
    Eigen::Vector6d force;

    // TODO: update the error value

    force = (_op_sp_inertia * _eddot) + (_D * _edot) + (_K * _e);

    return force;
}

// ---------------------------------------- SETTER && GETTER ---------------------------------------- //

void CartesianImpedanceController::set_stiffness_damping(const Eigen::Matrix6d &newK_diag, const Eigen::Matrix6d &newD_diag)
{
    // TODO: add cout check
    _K_diag = newK_diag;
    _D_diag = newD_diag;
}

Eigen::Matrix6d CartesianImpedanceController::get_stiffness() const
{
    return _K_diag;
}

Eigen::Matrix6d CartesianImpedanceController::get_damping() const
{
    return _D_diag;
}

void CartesianImpedanceController::set_reference_value(Eigen::Vector6d acc_ref, Eigen::Vector6d vel_ref, Eigen::Vector6d pos_ref)
{
    _xddot_ref = acc_ref;
    _xdot_ref = vel_ref;
    _x_ref = pos_ref;
}











