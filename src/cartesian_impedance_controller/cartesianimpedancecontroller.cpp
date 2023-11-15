#include "cartesianimpedancecontroller.h"


// ==============================================================================
// Constructor and Destructor
// ==============================================================================

CartesianImpedanceController::CartesianImpedanceController(ModelInterface::Ptr model,
                                                           RobotChain& leg,
                                                           Eigen::Matrix6d stiffness):
    _model(model),
    _leg(leg),
    _K(stiffness)
{
    _end_effector_link = _leg.getTipLinkName();
    _root_link = _leg.getBaseLinkName();

    _model->getRelativeJacobian(_end_effector_link, _root_link, _J);

    if (_J.cols() != 46 || _J.rows() != 6)
        cout << "[ERROR]: strange Jacobian dimension " << _J.rows() << " - " << _J.cols() << endl;

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

    // Filter
    //_velocity_filter = SignProcUtils::MovAvrgFilt(6,_dt,15);

    cout << "[OK]: impedance controller correctly constructed!" << endl;
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

    // Check matrix dimension compatibility
    try {

        if(_J.cols() != _B_inv.rows()){
            string msg = string("Cols of J ") + to_string(_J.cols()) + string(", Rows of B_inv ") + to_string(_B_inv.rows());
            throw std::runtime_error(msg);
        }

        if(_B_inv.cols() != _J.transpose().rows()){
            string msg = string("Cols of B_inv ") + to_string(_B_inv.cols()) + string(", Rows of J^T ") + to_string(_J.transpose().rows());
            throw std::runtime_error(msg);
        }

        // Λ = (J * B¯¹ * J)¯¹
        _op_sp_inertia = _J * _B_inv * _J.transpose();

        _op_sp_inertia = _op_sp_inertia.inverse();

    } catch (const std::exception& e) {
        std::cerr << "[ERROR]: incompatible matrix dimension: " << e.what() << endl;
    }

    // Cholesky decomposition
    try {
        cholesky_decomp(_op_sp_inertia);  // Store the resulting matrix in the variable _Q;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR]: " << e.what() << endl;
    }

}

void CartesianImpedanceController::update_K_omega()
{
    // Check matrix dimension compatibility

    if (_Q.size()!=_K.size())
        cout << "[ERROR]: Matrix Q and K do not have compatible matrix" << endl;

    _K_omega = _Q.transpose() * _K * _Q.transpose();

    cout << _K << endl;


}

void CartesianImpedanceController::update_D()
{
    // Check matrix dimension compatibility
    try {
        if(_Q.cols() != _D_zeta.rows()){
            string msg = string("Cols of Q ") + to_string(_Q.cols()) + string(", Rows of D_zeta ") + to_string(_D_zeta.rows());
            throw std::runtime_error(msg);
        }

        if(_D_zeta.cols() != _K_omega.rows()){
            string msg = string("Cols of D_zeta ") + to_string(_D_zeta.cols()) + string(", Rows of K_omega ") + to_string(_K_omega.rows());
            throw std::runtime_error(msg);
        }

        if(_K_omega.cols() != _Q.transpose().rows()){
            string msg = string("Cols of K_omega ") + to_string(_K_omega.cols()) + string(", Rows of Q^T ") + to_string(_Q.transpose().rows());
            throw std::runtime_error(msg);
        }

        _D = 2 * _Q * _D_zeta * matrix_sqrt(_K_omega) * _Q.transpose();

    } catch (const std::exception& e) {
        std::cerr << "[ERROR]: incompatible matrix dimension: " << e.what() << endl;
    }
}

void CartesianImpedanceController::update_real_value()
{

    // Get position
    Eigen::Affine3d pose;
    _model->getPose(_end_effector_link, _root_link, pose);

    _x_real << pose.translation(), pose.rotation().eulerAngles(0, 1, 2);

    // Get velocity
    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

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

    _e = _x_real - _x_ref;  // Position error

    _edot = _xdot_real - _xdot_ref; // Velocity error

    //_eddot = _xddot_real - _xddot_ref; // Acceleration error

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

    // Check matrix dimension compatibility
    try {

//        if(_op_sp_inertia.cols() != _eddot.rows()){
//            string msg = string("Cols of Λ ") + to_string(_Q.cols()) + string(", Rows of eddot ") + to_string(_D_zeta.rows());
//            throw std::runtime_error(msg);
//        }

        if(_D.cols() != _edot.rows()){
            string msg = string("Cols of D ") + to_string(_D.cols()) + string(", Rows of edot ") + to_string(_edot.rows());
            throw std::runtime_error(msg);
        }

        if(_K.cols() != _e.rows()){
            string msg = string("Cols of K ") + to_string(_K.cols()) + string(", Rows of e ") + to_string(_e.rows());
            throw std::runtime_error(msg);
        }

        force = (_D * _edot) + (_K * _e);

        if (_J.transpose().cols()!= force.rows())
            throw std::runtime_error("Matrix dimensions are not compatible for multiplication in torque computation.");

        torque = _J.transpose() * force;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR]: incompatible matrix dimension: " << e.what() << endl;
    }

    return torque.tail(40);
}

Eigen::Matrix6d CartesianImpedanceController::matrix_sqrt(Eigen::Matrix6d matrix)
{
    return matrix.array().sqrt();
}

void CartesianImpedanceController::cholesky_decomp(Eigen::Matrix6d matrix)
{

    Eigen::LLT<Eigen::MatrixXd> cholesky_of_A(matrix);

    // Check if the decomposition was successful
    if(cholesky_of_A.info() == Eigen::Success)

        _Q = cholesky_of_A.matrixL();

    else

        std::cerr << "[ERROR]: Cholesky decomposition failed!" << std::endl;


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

    Eigen::Affine3d pose;
    _model->getPose(_end_effector_link, _root_link, pose);

    _x_ref << pose.translation(), pose.rotation().eulerAngles(0, 1, 2);

    _model->getRelativeVelocityTwist(_end_effector_link, _root_link, _xdot_real);

}

void CartesianImpedanceController::setEnd_effector_link(const string &newEnd_effector_link)
{
    _end_effector_link = newEnd_effector_link;
}


