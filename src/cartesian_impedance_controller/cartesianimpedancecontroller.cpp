#include "cartesianimpedancecontroller.h"


// ==============================================================================
// Constructor and Destructor
// ==============================================================================

CartesianImpedanceController::CartesianImpedanceController(ModelInterface::Ptr model,
                                                           Eigen::Matrix6d stiffness,
                                                           const string end_effector):
    _model(model),
    _K(stiffness),
    _end_effector_link(end_effector)
{

//    _end_effector_link = _leg.getTipLinkName();
//    _root_link = _leg.getBaseLinkName();

    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    _model->getInertiaInverse(_B_inv);

    _n_joints = _J.rows();

    // Inizialize all parameteres to zero
    _xdot_real = _xdot_prec = Eigen::Vector6d::Zero();
    _edot = _e = Eigen::Vector6d::Zero();

    // Initialize all matrix to identity matrix
    //_B_inv = Eigen::MatrixXd::Identity(_n_joints, _n_joints);
    _Q = Eigen::Matrix6d::Identity();
    _K_omega = _D_zeta = _D = _op_sp_inertia = Eigen::Matrix6d::Identity();

    // Setting the derivation time
    _dt = 0.01; //NOTE: since we are in real-time, is it correct to have a dt?

    //Debug
    logger = XBot::MatLogger2::MakeLogger("/tmp/logger.mat");


    // Filter
    //_velocity_filter = SignProcUtils::MovAvrgFilt(6,_dt,15);

    //NOTE: Debug print
    //cout << "[OK]: impedance controller for " << _leg.getChainName() << " correctly constructed!" << endl;
    cout << "Configuration parameters:" << endl;
    cout << "Root link: " << _root_link << endl << "End effector link: " << _end_effector_link << endl;
    cout << "Stiffness\n" << _K << endl;
    cout << "Damping\n" << _D_zeta << endl;
    //cout << "Joint space inertial matrix\n" << _B_inv << endl;
    //cout << "Jacobian transpose:\n" << _J.transpose() << endl;    //it is correct, good job guys
    cout << "==================" << endl;

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

    cout << _end_effector_link << endl;

    // NOTE: Debug print
    //cout << _J.transpose() << "\n----------------------" << endl;

    // Λ = (J * B¯¹ * J^T)¯¹
    _op_sp_inertia = _J * _B_inv * _J.transpose();

    //isPositiveDefinite(_op_sp_inertia);

    _op_sp_inertia = svd_inverse(_op_sp_inertia);

//    cout << "Lambda:\n" << _op_sp_inertia << endl;
//    cout << "Eigenvalues: " << _op_sp_inertia.eigenvalues() << "\n----------" << endl;
    cout << "Lambda:\n" <<_op_sp_inertia << endl;

    // WARNING
    cholesky_decomp(_K, _op_sp_inertia);  // Store the resulting matrix in the variable _Q;
}

void CartesianImpedanceController::update_D()
{

    _D = _Q * _D_zeta * matrix_sqrt(_K_omega) * _Q.transpose();
    //_D = Eigen::Matrix6d::Identity() * 50;
    //isPositiveDefinite(_D);

    // NOTE: Debug print
//    cout << _leg.getChainName() << endl;
    cout << "D:\n" <<_D << endl;
//    cout << "------" << endl;

}

void CartesianImpedanceController::update_real_value()
{

    // Get position
    _model->getPose(_end_effector_link, _root_link, _x_real);

    // Get velocity
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);
    logger->add("real_vel", _xdot_real);


//    // Get acceleration
//    //_model->getRelativeAccelerationTwist(_end_effector_link, _root_link, _xddot_real);
//    Eigen::VectorXd vect = _xdot_real.head(6); // casting the data in a VectorXd object needed for the filter function
//    _velocity_filter.add_sample(vect);
//    _velocity_filter.get(vect);  // filtering velocity of the CoM
//    _xdot_real = vect.head(6);

//    _xddot_real = (_xdot_real - _xdot_prec) / _dt;

}

void CartesianImpedanceController::compute_error()
{

    // Position error
    _e << _x_real.translation() - _x_ref.translation(), orientation_error();

    // Velocity error
    _edot = _xdot_real;

    //_eddot = _xddot_real - _xddot_ref; // Acceleration error

    //NOTE: Debug print
//    cout << _leg.getChainName() << endl;
    cout << "Pos error:\n" << _e << endl;
    cout << "Vel error:\n" << _edot << endl;

}

Eigen::Vector3d CartesianImpedanceController::orientation_error(){

    Eigen::Matrix3d rotational_error = _x_ref.rotation() * _x_real.rotation().transpose();

    Eigen::AngleAxisd angleAxis(rotational_error);
    Eigen::Vector3d axis = angleAxis.axis();    // axis about which is exected the rotation
    double angle = angleAxis.angle();   // in radiant

    return axis * angle;

}

Eigen::VectorXd CartesianImpedanceController::compute_torque()
{

    update_inertia();
    update_D();   //WARNING
    update_real_value();
    compute_error();

    // TODO: create private variable and inizialize them in the constructor
    Eigen::Vector6d force;
    Eigen::VectorXd torque;

    force = (_D * _edot) + (_K * _e);

    //NOTE: Debug print
//    cout << _leg.getChainName() << endl;
    cout << "Force:\n" << force << endl;

    torque = _J.transpose() * force;

    //NOTE: Debug print
//    cout << torque.transpose() << endl;

    return torque.tail(40);
}

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{

    Eigen::Vector6d diag = matrix.diagonal();
    diag = (diag.array() + 0.01).cwiseSqrt();

    return diag.asDiagonal();

}

void CartesianImpedanceController::isPositiveDefinite(const Eigen::MatrixXd& matrix) {

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(matrix);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();

    if (eigenvalues.minCoeff() > 0){
        cout << "Is positive definite:\n" << matrix << endl;
    }
    else if (eigenvalues.minCoeff() >= 0) {
        cout << "Is positive semidefinite:\n" << matrix << endl;

    }else if (eigenvalues.minCoeff() < 0) {
        cout << "Not positive definite or semidefinite: " << eigenvalues.minCoeff() << "\n" << matrix << endl;

    }

}

Eigen::Matrix6d CartesianImpedanceController::cholesky_decomp(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{

    //TODO: create private variable and inizialize them in the constructor

    // Matrix A will be the stiffness _K
    // Matrix B will be Lambda the operational space inertia matrix

    //isPositiveDefinite(B)

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_B(B);

    Eigen::MatrixXd Phi_B = eigen_solver_B.eigenvectors();
    Eigen::VectorXd lambda_B = eigen_solver_B.eigenvalues().array().sqrt().array() + 0.0001;

    Eigen::MatrixXd Lambda_B_sqrt = lambda_B.asDiagonal();

    Eigen::MatrixXd Phi_B_hat = Phi_B * Lambda_B_sqrt.inverse();

    Eigen::MatrixXd A_hat = Phi_B_hat.transpose() * A * Phi_B_hat;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_A(A_hat);

    Eigen::MatrixXd Phi_A = eigen_solver_A.eigenvectors();

    Eigen::MatrixXd Phi = Phi_B_hat * Phi_A;    // What i find here is X = (Q^T)^-1, I need to know Q = (X^-1)^T

    _K_omega = eigen_solver_A.eigenvalues().asDiagonal();
    _Q = Phi.inverse().transpose();

    //NOTE: Debug print
//    cout << _leg.getChainName() << endl;
    cout << "K_omega:\n" << _K_omega << endl;
    cout << "Q * Q^T:\n" <<_Q * _Q.transpose() << endl;
//    cout << "=====" << endl;

    return _Q;

}

Eigen::Matrix6d CartesianImpedanceController::svd_inverse(Eigen::Matrix6d matrix){

    // Compute the SVD decomposition -> A = USV^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

    _U = svd.matrixU(); // Left singular vectors
    _V = svd.matrixV(); // Right singular vectors
    _S = svd.singularValues(); // Singular values matrix

    // NOTE: Debug print
//    cout << "Singular values original:\n" << _S.transpose() << endl;
//    cout << "U:\n" << _U << endl;
//    cout << "V:\n" << _V << endl;


    for (int i = 0; i < _S.rows(); i++){

        _S_pseudo_inverse(i) = f(_S(i));

    }

    //NOTE: Debug print
//    cout << "Singular values modified:\n" << _S_pseudo_inverse.transpose() << endl;

    return _U * _S_pseudo_inverse.asDiagonal() * _U.transpose();
}

double CartesianImpedanceController::f(double x){

    return (x+0.01)/(_rho + pow(x, 2));

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

void CartesianImpedanceController::set_reference_value()
{

    _model->getPose(_end_effector_link, _root_link, _x_ref);
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

}

void CartesianImpedanceController::setEnd_effector_link(const string &newEnd_effector_link)
{
    _end_effector_link = newEnd_effector_link;
}

void CartesianImpedanceController::reset_logger(){
    logger.reset();
}

