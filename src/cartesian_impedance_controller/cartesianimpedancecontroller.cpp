#include "cartesianimpedancecontroller.h"


// ==============================================================================
// Constructor
// ==============================================================================

CartesianImpedanceController::CartesianImpedanceController(ModelInterface::Ptr model,
                                                           Eigen::Matrix6d stiffness,
                                                           const string end_effector,
                                                           const string base_link):
    _model(model),
    _K(stiffness),
    _end_effector_link(end_effector),
    _root_link(base_link)
{

    // Inizialize Jacobian and Joint inertia matrix
    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    _model->getInertiaMatrix(_B_inv);

    // Inizialize all parameteres to zero
    _xdot_real = _xdot_prec = Eigen::Vector6d::Zero();
    _edot = _e = Eigen::Vector6d::Zero();

    // Initialize all matrix to identity matrix
    _Q = _K_omega = _D = _op_sp_inertia = Eigen::Matrix6d::Identity();

    _D_zeta = Eigen::Matrix6d::Identity();  //TODO: set this parameter through YAML file

    // SVD decompositin variable initialization
    _U = _V = Eigen::MatrixXd::Identity(6,6);
    _S = _S_pseudo_inverse = Eigen::VectorXd::Zero(6);

    // Other initialization
    _rotational_error = Eigen::Matrix3d::Identity();
    _axis = Eigen::Vector3d::Zero();
    _angle = 0.0;
    _diag = Eigen::Vector6d::Zero();

    //Print of configuration parameters
    cout << "Configuration parameters:" << endl;
    cout << "Root link: " << _root_link << endl << "End effector link: " << _end_effector_link << endl;
    cout << "Stiffness\n" << _K << endl;
    cout << "Damping\n" << _D_zeta << endl;
    cout << "==================" << endl;

    //Debug
    //logger = XBot::MatLogger2::MakeLogger("/home/riccardo/Documents/MATLAB/logger.mat");
}

// ==============================================================================
// Additional Functions
// ==============================================================================

void CartesianImpedanceController::update_inertia()
{

    // Update Jacobian and Joint inertia since they are configuration dependant
    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    _model->getInertiaMatrix(_B_inv);

    // Λ = (J * B¯¹ * J^T)¯¹
    _op_sp_inertia = _J * _B_inv.inverse() * _J.transpose();

    /*
     * controlling the legs will cause a singularity in the Jacobian (no joint on the wheel),
     * resulting in a non invertible task space inertia matrix. A possible solution is to
     * invert the matrix using the inverse of the SVD decomposition (that since Λ is symmetric
     * is equal to a spectral theorem UVU^T), quick and dirty method
    */
    _op_sp_inertia = svd_inverse(_op_sp_inertia);

    // NOTE: Debug print
    //cout << "Lambda:\n" << _op_sp_inertia << endl;

    Q_computation(_K, _op_sp_inertia);  // will automatically store the resulting matrix in the variable _Q;
}

void CartesianImpedanceController::update_D()
{

    // Refer to documentation for the formula
    _D = 2 * _Q * _D_zeta * matrix_sqrt(_K_omega) * _Q.transpose();

    isPositiveDefinite(_D);

}

void CartesianImpedanceController::update_real_value()
{

    // Get position
    _model->getPose(_end_effector_link, _root_link, _x_real);

    // Get velocity
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

}

void CartesianImpedanceController::compute_error()
{

    // Reset position and velocity error
    _e = Eigen::Vector6d::Zero();
    _edot = Eigen::Vector6d::Zero();

    // Position error
    _e << _x_ref.translation() - _x_real.translation(), orientation_error();

    // Velocity error
    _edot = -_xdot_real;

    // Debug print
    //cout << "Pos error:\n" << _e << endl;
    //cout << "Vel error:\n" << _edot << endl;

}

Eigen::Vector3d CartesianImpedanceController::orientation_error(){

    _rotational_error = _x_ref.rotation() * _x_real.rotation().transpose();

    Eigen::AngleAxisd angleAxis(_rotational_error);
    _axis = angleAxis.axis();    // axis about which is executed the rotation
    _angle = angleAxis.angle();   // [rad]

    return _axis * _angle;

}

Eigen::VectorXd CartesianImpedanceController::compute_torque()
{

    update_inertia();
    update_D();
    update_real_value();
    compute_error();

    // Reset to zero each cycle
    _force = Eigen::Vector6d::Zero();
    _torque = Eigen::VectorXd::Zero(46);

    _force = (_D * _edot) + (_K * _e);

    _torque = _J.transpose() * _force;

    //Debug print
    //cout << torque.transpose() << endl;
    //cout << "Force:\n" << force << endl;

    // The model works with 46 joints (first 6 of the floating base), while the robot works with 40 joints
    // (exclude the floating base joints), so this returns all the torque excluding the floating base part

    return _torque.tail(40);
}

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{

    _diag = matrix.diagonal();
    _diag = (_diag.array() + 0.001).cwiseSqrt();  // add an offset terms to guarantee that the values are positive

    return _diag.asDiagonal();

}

void CartesianImpedanceController::isPositiveDefinite(const Eigen::MatrixXd& matrix) {

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(matrix);
    _eigenvalues = eigen_solver.eigenvalues();

    if (_eigenvalues.minCoeff() > 0){
        //cout << "Is positive definite:\n" << matrix << endl;
    }
    else if (_eigenvalues.minCoeff() >= 0) {
        cout << "Is positive semidefinite:\n" << matrix << endl;

    }else if (_eigenvalues.minCoeff() < 0) {
        cout << "Not positive definite or semidefinite: " << _eigenvalues.minCoeff() << "\n" << matrix << endl;

    }

}

Eigen::Matrix6d CartesianImpedanceController::Q_computation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{

    // Matrix A will be the stiffness _K
    // Matrix B will be Lambda the operational space inertia matrix

    //isPositiveDefinite(B) // check the positive definiteness

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_B(B);

    _Phi_B = eigen_solver_B.eigenvectors();

    _lambda_B = eigen_solver_B.eigenvalues().array().sqrt().array();

    _Lambda_B_sqrt = _lambda_B.asDiagonal();

    _Phi_B_hat = _Phi_B * _Lambda_B_sqrt.inverse();

    _A_hat = _Phi_B_hat.transpose() * A * _Phi_B_hat;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_A(_A_hat);

    _Phi_A = eigen_solver_A.eigenvectors();

    _Phi = _Phi_B_hat * _Phi_A;    // What i find here is X = (Q^T)^-1, I need to know Q = (X^-1)^T

    _K_omega = eigen_solver_A.eigenvalues().asDiagonal();
    _Q = _Phi.inverse().transpose();

    // Debug print
    //cout << "Q:\n" << _Q << endl;
    //cout << "K_omega:\n" << _K_omega << endl;
    //cout << "K computed:\n" << _Q * _K_omega * _Q.transpose()<< endl;
    //cout << "Q * Q^T:\n" <<_Q * _Q.transpose() << endl;
    //cout << "=====" << endl;

    return _Q;

}

Eigen::Matrix6d CartesianImpedanceController::svd_inverse(Eigen::Matrix6d matrix){

    // Compute the SVD decomposition -> A = USV^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

    _U = svd.matrixU(); // Left singular vectors
    _V = svd.matrixV(); // Right singular vectors
    _S = svd.singularValues(); // Singular values matrix

    // Debug print
    //cout << "Singular values original:\n" << _S.transpose() << endl;
    //cout << "U:\n" << _U << endl;
    //cout << "V:\n" << _V << endl;

    /*
     * The pseudo inverse of the singular values matrix is computed normally as diag = {1/s_i},
     * where s_i are the singular values. Due to the singularity in this case, a singular value
     * will be 0, causing the pseudo inverse to diverge to infinite on one element in the diagonal.
     * To avoid this problem, here is used a function f(x) that send to zero any value that
     * could cause the matrix to diverge
    */

    for (int i = 0; i < _S.rows(); i++){

        _S_pseudo_inverse(i) = f(_S(i));

    }

    // Λ^† = (V * S^† * U^T) = (U * S^† * U^T) since Λ is symmetric
    return _U * _S_pseudo_inverse.asDiagonal() * _U.transpose();    // as said from
}

double CartesianImpedanceController::f(double x){

    /*
     *          (x + offset)
     *  f(x) = -------------    (Plot it if you want)
     *           rho + x²
     *
     *  The offset value is used to guarantee positive definiteness of the inverse matrix
    */

    return (x + _offset)/(_rho + pow(x, 2));

}

// ==============================================================================
// Setter and Getter
// ==============================================================================

void CartesianImpedanceController::set_reference_value()
{

    _model->getPose(_end_effector_link, _root_link, _x_ref);
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

}

void CartesianImpedanceController::reset_logger(){
    logger.reset();
}

