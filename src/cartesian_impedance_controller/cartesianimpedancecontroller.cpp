#include "cartesianimpedancecontroller.h"


// ==============================================================================
// Constructor
// ==============================================================================

CartesianImpedanceController::CartesianImpedanceController(ModelInterface::Ptr model,
                                                           Eigen::Matrix6d stiffness,
                                                           Eigen::Matrix6d damping_factor,
                                                           const string end_effector,
                                                           const string base_link,
                                                           string task_name):
    _model(model),
    _K(stiffness),
    _D_zeta(damping_factor),
    _end_effector_link(end_effector),
    _root_link(base_link),
    _task_name(task_name)
{
    // Inizialize Jacobian and Joint inertia matrix
    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    _B_inv = Eigen::MatrixXd::Zero(46,46);
    // _model->getInertiaMatrix(_B_inv);
    // update_inertia();

    // Inizialize all parameteres to zero
    _x_real = _x_ref = Eigen::Affine3d::Identity();
    _xdot_real = _xdot_ref = Eigen::Vector6d::Zero(6);
    _xddot_ref = Eigen::Vector6d::Zero(6);
    _edot = _e = Eigen::Vector6d::Zero(6);

    // Initialize all matrix to identity matrix
    _Q = _K_omega = _D = _op_sp_inertia = Eigen::Matrix6d::Identity();

    // SVD decompositin variable initialization
    _U = _V = Eigen::MatrixXd::Identity(6,6);
    _S = _S_pseudo_inverse = Eigen::VectorXd::Zero(6);

    // Other initialization
    _rotational_error = Eigen::Matrix3d::Identity(3, 3);
    _axis = Eigen::Vector3d::Zero();
    _angle = 0.0;
    _diag = _eigenvalues = Eigen::Vector6d::Zero();

    // Controller
    _force = Eigen::Vector6d::Zero(6);
    _torque = Eigen::VectorXd::Zero(46);

    // SVD initialization
    _U = _V = Eigen::Matrix6d::Identity(6, 6);
    _S = _S_pseudo_inverse = Eigen::Vector6d::Zero(6);

    // Generalized eigenvalue problem variables initialization
    _Phi = _Phi_A = _Phi_B = _Phi_B_hat = _A_hat = _Lambda_B_sqrt = Eigen::Matrix6d::Identity(6, 6);
    _lambda_B = Eigen::Vector6d::Zero(6);

    //Print of configuration parameters
    print_config_param();

    // Initialization of the ROS node for log purpose
    _ros = make_unique<RosSupport>(ros::NodeHandle(_task_name));
    _stats_publisher = _ros->advertise<riccardo_giacchino_master_thesis::CartesianController>(string("cartesian_controller"), 1);
    _msg.name = _end_effector_link;

}

// ==============================================================================
// Additional Functions
// ==============================================================================

void CartesianImpedanceController::update_inertia()
{

    // Update Jacobian and Joint inertia matrix since they are configuration dependant
    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);
    _model->getInertiaMatrix(_B_inv);

    // Λ = (J * B¯¹ * J^T)¯¹
    _op_sp_inertia.noalias() = _J * _B_inv.llt().solve(_J.transpose());

    /* Make a controller on the legs will cause a singularity in the Jacobian (no joint on the wheel),
     * resulting in a non invertible task space inertia matrix. A possible solution is to
     * invert the matrix using the inverse of the SVD decomposition (since Λ is symmetric
     * is equal to a spectral theorem UVU^T), quick and dirty method.
    */
    _op_sp_inertia.noalias() = svd_inverse(_op_sp_inertia);



    // --------------- DEBUG ---------------
    //cout << "Lambda:\n" << _op_sp_inertia << endl;

    Q_computation(_K, _op_sp_inertia);  // will automatically store the resulting matrix in the variable _Q;
}

void CartesianImpedanceController::update_D()
{

    // Refer to documentation for the formula
    _D.noalias() = 2 * _Q * _D_zeta * matrix_sqrt(_K_omega) * _Q.transpose();

    // --------------- DEBUG ---------------
    // cout << string(_end_effector_link + "\n")  << _K_omega << endl << _Q.transpose() << endl;

    //isPositiveDefinite(_D); // checks if the matrix is positive definite

}

void CartesianImpedanceController::update_real_value()
{

    // Get position
    _model->getPose(_end_effector_link, _root_link, _x_real);

    // Get velocity
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

    // --------------- LOGGER ---------------
    tf::poseEigenToMsg(_x_real, _msg.real_position);
    tf::twistEigenToMsg(_xdot_real, _msg.real_velocity);

}

void CartesianImpedanceController::compute_error()
{

    // Reset position and velocity error
    _e.setZero();
    _edot.setZero();

    // Position error
    _e << _x_ref.translation() - _x_real.translation(), orientation_error();

    // Velocity error
    _edot.noalias() = _xdot_ref - _xdot_real;

    // Debug print
    //cout << "Pos error:\n" << _e << endl;
    //cout << "Vel error:\n" << _edot << endl;

    // --------------- LOGGER ---------------
    tf::twistEigenToMsg(_e, _msg.position_error);
    tf::twistEigenToMsg(_edot, _msg.velocity_error);

}

Eigen::Vector3d CartesianImpedanceController::orientation_error(){

    // Check documentation for formula
    _rotational_error.noalias() = _x_ref.rotation() * _x_real.rotation().transpose();

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
    _force.setZero();
    _torque.setZero();

    // Cartesian controller law equation
    _force.noalias() = (_op_sp_inertia * _xddot_ref) + (_D * _edot) + (_K * _e);

    // τ = J^T * F
    _torque.noalias() = _J.transpose() * _force;

    // --------------- DEBUG ---------------
    //cout << _torque.transpose() << endl;
    //cout << "Force:\n" << _force << endl;

    // --------------- LOGGER ---------------
    tf::wrenchEigenToMsg(_force, _msg.force);
    vectorEigenToMsg();

    // Publish the message on the topic
    _stats_publisher->publish(_msg);

    return _torque;
}

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{

    _diag = matrix.diagonal();
    _diag = (_diag.array() + _rho).cwiseSqrt();  // add an offset terms to guarantee that the values are positive

    return _diag.asDiagonal();

}

void CartesianImpedanceController::isPositiveDefinite(const Eigen::Matrix6d& matrix) {

    // Compute eigenvalues
    _eigen_solver.compute(matrix);
    _eigenvalues = _eigen_solver.eigenvalues();

    // Check if there are any eigenvalues lower than zero
    if (_eigenvalues.minCoeff() > 0){
        //cout << "Is positive definite:\n" << matrix << endl;
    }
    else if (_eigenvalues.minCoeff() >= 0) {
        cout << "Is positive semidefinite:\n" << matrix << endl;

    }else if (_eigenvalues.minCoeff() < 0) {
        cout << "Not positive definite or semidefinite: " << _eigenvalues.minCoeff() << "\n" << matrix << endl;

    }

}

Eigen::Matrix6d CartesianImpedanceController::Q_computation(const Eigen::Matrix6d& A, const Eigen::Matrix6d& B)
{

    // Matrix A will be the stiffness _K
    // Matrix B will be Lambda the operational space inertia matrix

    //isPositiveDefinite(B) // check the positive definiteness

    _eigen_solver.compute(B);

    _Phi_B = _eigen_solver.eigenvectors();

    _lambda_B = _eigen_solver.eigenvalues().array().sqrt().array();

    _Lambda_B_sqrt = _lambda_B.asDiagonal();

    _Phi_B_hat.noalias() = _Phi_B * _Lambda_B_sqrt.inverse();

    _A_hat.noalias() = _Phi_B_hat.transpose() * A * _Phi_B_hat;

    _eigen_solver.compute(_A_hat);

    _Phi_A = _eigen_solver.eigenvectors();

    _Phi.noalias() = _Phi_B_hat * _Phi_A;    // What i find here is X = (Q^T)^-1, I need to know Q = (X^-1)^T

    _K_omega = _eigen_solver.eigenvalues().asDiagonal();
    _Q.noalias() = _Phi.inverse().transpose();

    // --------------- DEBUG ---------------
    //cout << "Q:\n" << _Q << endl;
    //cout << "K_omega:\n" << _K_omega << endl;
    //cout << "K computed:\n" << _Q * _K_omega * _Q.transpose()<< endl;
    //cout << "Q * Q^T:\n" <<_Q * _Q.transpose() << endl;
    //cout << "=====" << endl;

    return _Q;

}

Eigen::Matrix6d CartesianImpedanceController::svd_inverse(Eigen::Matrix6d matrix){

    // Compute the SVD decomposition -> A = USV^T
    _svd.compute(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

    _U = _svd.matrixU(); // Left singular vectors
    _V = _svd.matrixV(); // Right singular vectors
    _S = _svd.singularValues(); // Singular values matrix

    // --------------- DEBUG ---------------
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
    return _U * _S_pseudo_inverse.asDiagonal() * _U.transpose();
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

void CartesianImpedanceController::print_config_param(){

    cout << "[OK] Successfully created controller for task " << _task_name << endl;
    cout << "CONFIGURATION PARAMETERS:" << endl;
    cout << "- ROOT LINK: " << _root_link << endl;
    cout << "- END EFFECTOR LINK: " << _end_effector_link << endl;
    cout << "- STIFFNESS: [" << _K.diagonal().transpose() << "]" << endl;
    cout << "- DAMPING: [" << _D_zeta.diagonal().transpose() << "]" << endl;
    cout << "=====================================================" << endl;

}

void CartesianImpedanceController::vectorEigenToMsg(){

    for (int i = 0; i < _torque.size(); i++){
        _msg.torque[i] = _torque(i);
    }

    for (int i = 0; i < _K_omega.diagonalSize(); i++){
        _msg.K_omega[i] = _K_omega.diagonal()(i);
    }

}

// ==============================================================================
// Setter and Getter
// ==============================================================================

void CartesianImpedanceController::set_reference_value(Eigen::Affine3d Tref, Eigen::Vector6d xdot_ref, Eigen::Vector6d xddot_ref)
{
    _x_ref = Tref;
    _xdot_ref = xdot_ref;
    _xddot_ref = xddot_ref;

    tf::poseEigenToMsg(_x_ref, _msg.reference_position);
    tf::twistEigenToMsg(_xdot_ref, _msg.reference_velocity);
    tf::twistEigenToMsg(_xddot_ref, _msg.reference_acceleration);
}

void CartesianImpedanceController::set_stiffness(Eigen::Matrix6d stiffness){

    _K = stiffness;

}

void CartesianImpedanceController::set_damping_factor(Eigen::Matrix6d damping_factor){

    _D_zeta = damping_factor;

}


Eigen::Matrix6d CartesianImpedanceController::get_Mass(){

    return _op_sp_inertia;

}

