#include "cartesianimpedancecontroller.h"


// ==============================================================================
// Real-time functions
// ==============================================================================

bool CartesianImpedanceController::on_initialize(){

    //TODO: inizialize variable

    _xddot_ref = _xdot_ref = _x_ref = Eigen::Vector6d::Zero();
    _xddot_real = _xdot_real = _xdot_prec = _x_real = Eigen::Vector6d::Zero();
    _eddot = _edot = _e = Eigen::Vector6d::Zero();

    _J = _B_inv = _Q = Eigen::MatrixXd::Identity();

    _K_omega = _K = _D_zeta = _D = _op_sp_inertia = Eigen::Matrix6d::Identity();

    _dt = 0.01; //NOTE: since we are in real-time, is it correct to have a dt?

    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);

    _n_joints = _J.rows();

    //TODO: manage control mode

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
    //TODO: make it more robust
    _K = _Q * _K_omega * _Q.transpose();
}

void CartesianImpedanceController::update_D()
{
    //TODO: make it more robust
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

// ==============================================================================
// Setter and Getter
// ==============================================================================

void CartesianImpedanceController::set_stiffness(const Eigen::Matrix6d &new_K)
{
    _K = new_K;
    // cout << "[OK]: New diagonal stiffness matrix added\n" << _K << endl;
}

void CartesianImpedanceController::set_stiffness()
{
    Eigen::Vector6d stiffness_value;

    stiffness_value << 1, 1, 1, 1, 1, 1;    // TODO: tmp values of the stiffness matrix, have to be changed

    _K = stiffness_value.asDiagonal();
    // cout << "[OK]: New diagonal stiffness matrix added\n" << _K << endl;

}

void CartesianImpedanceController::set_reference_value(Eigen::Vector6d& acc_ref, Eigen::Vector6d& vel_ref, Eigen::Vector6d& pos_ref)
{
    _xddot_ref = acc_ref;
    _xdot_ref = vel_ref;
    _x_ref = pos_ref;
    //cout << "[OK]: Set new reference value:\n" << _xddot_ref << endl << _xdot_ref << endl << _x_ref << endl;
}

void CartesianImpedanceController::setEnd_effector_link(const string &newEnd_effector_link)
{
    _end_effector_link = newEnd_effector_link;
}


