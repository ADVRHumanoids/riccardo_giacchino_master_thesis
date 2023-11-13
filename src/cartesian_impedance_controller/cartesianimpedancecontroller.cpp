#include "cartesianimpedancecontroller.h"


// ==============================================================================
// Constructor and Destructor
// ==============================================================================

CartesianImpedanceController::CartesianImpedanceController(ModelInterface::Ptr model,
                                                           const string end_effector_link_name,
                                                           const string root_link_name):
    _model(model),
    _end_effector_link(end_effector_link_name),
    _root_link(root_link_name)
{

    // Get the corresponding Jacobian
    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);

    // Computing the number of joints in the system
    _n_joints = _J.rows();

    // Inizialize all parameteres to zero
    _xddot_ref = _xdot_ref = _x_ref = Eigen::Vector6d::Zero();
    _xddot_real = _xdot_real = _xdot_prec = _x_real = Eigen::Vector6d::Zero();
    _eddot = _edot = _e = Eigen::Vector6d::Zero();

    // Initialize all matrix to identity matrix
    _B_inv = Eigen::MatrixXd::Identity(_n_joints, _n_joints);
    _Q = Eigen::Matrix6d::Identity();
    _K_omega = _K = _D_zeta = _D = _op_sp_inertia = Eigen::Matrix6d::Identity();

    // Setting the derivation time
    _dt = 0.01; //NOTE: since we are in real-time, is it correct to have a dt?

}

CartesianImpedanceController::CartesianImpedanceController(ModelInterface::Ptr model,
                                                           const string end_effector_link_name,
                                                           const string root_link_name,
                                                           Eigen::Matrix6d stiffness):
    _model(model),
    _end_effector_link(end_effector_link_name),
    _root_link(root_link_name),
    _K(stiffness)
{

    // Get the corresponding Jacobian
    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);

    // Computing the number of joints in the system
    _n_joints = _J.rows();

    // Inizialize all parameteres to zero
    _xddot_ref = _xdot_ref = _x_ref = Eigen::Vector6d::Zero();
    _xddot_real = _xdot_real = _xdot_prec = _x_real = Eigen::Vector6d::Zero();
    _eddot = _edot = _e = Eigen::Vector6d::Zero();

    // Initialize all matrix to identity matrix
    _B_inv = Eigen::MatrixXd::Identity(_n_joints, _n_joints);
    _Q = Eigen::Matrix6d::Identity();
    _K_omega = _D_zeta = _D = _op_sp_inertia = Eigen::Matrix6d::Identity();

    // Setting the derivation time
    _dt = 0.01; //NOTE: since we are in real-time, is it correct to have a dt?
}

// ==============================================================================
// Additional Functions
// ==============================================================================

void CartesianImpedanceController::update_model(ModelInterface::Ptr model)
{
    _model = model;
}

void CartesianImpedanceController::update_inertia()
{

    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);    // the Jacobian is configuration dependant, so it has to be updated every cycle
    _model->getInertiaInverse(_B_inv);  // compute the inertia matrix B, also configuration dependant

    // Λ = (J * B¯¹ * J)¯¹
    _op_sp_inertia = _J * _B_inv * _J.transpose();

    _op_sp_inertia = _op_sp_inertia.inverse();

    // Cholesky decomposition
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

Eigen::VectorXd CartesianImpedanceController::compute_torque()
{

    update_inertia();
    update_K_omega();
    update_D();
    update_real_value();
    compute_error();

    Eigen::Vector6d force;
    Eigen::VectorXd torque;

    force = (_op_sp_inertia * _eddot) + (_D * _edot) + (_K * _e);


    try {
        // Check if the dimensions are compatible for matrix multiplication
        if (_J.transpose().cols()!= force.rows())
            throw std::runtime_error("Matrix dimensions are not compatible for multiplication in torque computation.");

        torque = _J.transpose() * force;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR]: " << e.what() << std::endl;
        // Handle the exception (e.g., stop the execution or provide a default value for torque)
    }

    return force;
}

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{
    return matrix.array().sqrt();
}

void CartesianImpedanceController::cholesky_decomp(Eigen::Matrix6d matrix)
{
    //WARNING: are you sure that this compute the cholesky decomposition? Maybe we have to compute it manually
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(matrix);

    if (eigensolver.info() != Eigen::Success)
        throw std::runtime_error("Impossible the compute the Cholesky decomposition");

    // Get the eigenvectors in Q and eigenvalues in Lambda
    _Q = eigensolver.eigenvectors();

}

// ==============================================================================
// Setter and Getter
// ==============================================================================

void CartesianImpedanceController::set_stiffness(const Eigen::Vector6d &new_K)
{
    _K = new_K.asDiagonal();
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


