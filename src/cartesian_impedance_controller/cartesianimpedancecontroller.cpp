#include "cartesianimpedancecontroller.h"

// WARNING: THIS VERSION IS NOT WORKING

// ---------------------------------------- CONSTRUCTOR ---------------------------------------- //

//CartesianImpedanceController::CartesianImpedanceController(ros::NodeHandle nh,
//                                                           double dt,
//                                                           const string root_link,
//                                                           const string end_effector_link):
//    _nh(nh),
//    _dt(dt),
//    _root_link(root_link),
//    _end_effector_link(end_effector_link)
//{
//    XBot::ConfigOptions xbot_cfg = XBot::ConfigOptionsFromParamServer(_nh);
//    _model = XBot::ModelInterface::getModel(xbot_cfg);

//    try
//    {
//        _robot = XBot::RobotInterface::getRobot(xbot_cfg);
//        _model->syncFrom(*_robot);
//        _robot->setControlMode(XBot::ControlMode::Position());  //TODO: check if the control mode Position is still correct
//        _robot->setControlMode(
//            {
//                {"j_wheel_1", XBot::ControlMode::Velocity()},
//                {"j_wheel_2", XBot::ControlMode::Velocity()},
//                {"j_wheel_3", XBot::ControlMode::Velocity()},
//                {"j_wheel_4", XBot::ControlMode::Velocity()}
//            });
//    }
//    catch (const std::exception& e)
//    {
//        // Setting homing postion of the robot
//        Eigen::VectorXd qhome;
//        _model->getRobotState("home", qhome);
//        _model->setJointPosition(qhome);
//        _model->update();
//        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);
//        cerr << "[ Error ]: " << e.what() << endl;
//    }

//    _op_sp_inertia = Eigen::Matrix6d::Identity();

//    // TODO: compute automatically the number of joint of the kinematic chain
//    // By defalt should be 7, that are the number of joints in a single leg

//    _velocity_filter = SignProcUtils::MovAvrgFilt(_n_joints = 7, _dt, 20.0);
//}

//// ---------------------------------------- DESTRUCTOR ---------------------------------------- //

//CartesianImpedanceController::~CartesianImpedanceController()
//{
//    cout << "[ END ]: The program is terminating..." << endl;
//}




// ==============================================================================
// Real-time functions
// ==============================================================================

bool CartesianImpedanceController::on_initialize(){

    //TODO: inizialize variable

    _xddot_ref = _xdot_ref = _x_ref = Eigen::Vector6d::Zero();
    _xddot_real = _xdot_real = _xdot_prec = _x_real = Eigen::Vector6d::Zero();
    _eddot = _edot = _e = Eigen::Vector6d::Zero();

    _J = _B_inv = _Q = Eigen::MatrixXd::Zero();
    _K_omega = _K = _D_zeta = _D = _op_sp_inertia = Eigen::Matrix6d::Zero();




    //TODO: manage control mode


    return true;
}

void CartesianImpedanceController::on_start(){
    //TODO: implement
    //set zero the impedance of the controller already implemented
    //save in some way the state of the robot in this moment of time
    //that has to be setted back again in the on_stop
}

void CartesianImpedanceController::on_stop()
{
    //TODO: implement
    //set back again the state of the robot that you changed in the on_start funcition
}



// ==============================================================================
// Additional Functions
// ==============================================================================

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{
    return matrix.array().sqrt();
}


void CartesianImpedanceController::cholesky_decomp(Eigen::Matrix6d matrix)
{

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(matrix);

    if (eigensolver.info() != Eigen::Success)
        throw std::runtime_error("Impossible the compute the Cholesky decomposition");

    // Get the eigenvectors in Q and eigenvalues in Lambda
    _Q = eigensolver.eigenvectors();

}

void CartesianImpedanceController::update_inertia()
{

    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    _model->getInertiaInverse(_B_inv);

    _op_sp_inertia = _J * _B_inv * _J.transpose();

    _op_sp_inertia = _op_sp_inertia.inverse();

    try {
        cholesky_decomp(_op_sp_inertia);  // Store the resulting matrix in the variable _Q;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR]: " << e.what() << endl;
    }

}

void CartesianImpedanceController::update_K_omega()
{
    _K = _Q * _K_omega * _Q.transpose();
}

void CartesianImpedanceController::update_D()
{
    _D = 2 * _Q * _D_zeta * matrix_sqrt(_K_omega) * _Q.transpose();
}

void CartesianImpedanceController::update_real_value()
{

    // Get position
    Eigen::Affine3d pose;
    _model->getPose(_end_effector_link, _root_link, pose);

    _x_real << pose.translation(), pose.rotation().eulerAngles(0, 1, 2);

    // Get velocity
    _xdot_prec = _xdot_real;
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

    // Get acceleration
    //_model->getRelativeAccelerationTwist(_end_effector_link, _root_link, _xddot_real);
    Eigen::VectorXd vect = _xdot_real.head(6); // casting the data in a VectorXd object needed for the filter function
    _velocity_filter.add_sample(vect);
    _velocity_filter.get(vect);  // filtering velocity of the CoM
    _xdot_real = vect.head(6);

    _xddot_real = (_xdot_real - _xdot_prec) / _dt;

}

void CartesianImpedanceController::compute_error()
{

    _e = _x_real - _x_ref;  // Position error

    _edot = _xdot_real - _xdot_ref; // Velocity error

    _eddot = _xddot_real - _xddot_ref; // Acceleration error

}

Eigen::Vector6d CartesianImpedanceController::compute_force()
{

    update_inertia();
    update_K_omega();
    update_D();
    update_real_value();
    compute_error();

    Eigen::Vector6d force;

    force = (_op_sp_inertia * _eddot) + (_D * _edot) + (_K * _e);

    return force;
}

// ==============================================================================
// Setter and Getter
// ==============================================================================

void CartesianImpedanceController::set_stiffness_damping(const Eigen::Matrix6d &newK, const Eigen::Matrix6d &newD_zeta)
{
    _K = newK;
    cout << "[OK]: New diagonal stiffness matrix added\n" << _K_omega << endl;

    _D_zeta = newD_zeta;
    cout << "[OK]: New diagonal damping matrix added" << _D_zeta << endl;
}

Eigen::Matrix6d CartesianImpedanceController::get_stiffness() const
{
    return _K;
}

Eigen::Matrix6d CartesianImpedanceController::get_damping() const
{
    return _D;
}

void CartesianImpedanceController::set_reference_value(Eigen::Vector6d acc_ref, Eigen::Vector6d vel_ref, Eigen::Vector6d pos_ref)
{
    _xddot_ref = acc_ref;
    _xdot_ref = vel_ref;
    _x_ref = pos_ref;
    cout << "[OK]: Set new reference value:\n" << _xddot_ref << endl << _xdot_ref << endl << _x_ref << endl;
}
