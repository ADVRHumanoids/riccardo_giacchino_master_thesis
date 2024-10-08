#ifndef CARTESIANIMPEDANCECONTROLLER_H
#define CARTESIANIMPEDANCECONTROLLER_H

// ==============================================================================
// Include
// ==============================================================================

#include <iostream>
#include <thread>
#include <math.h>
#include <chrono>

#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <matlogger2/matlogger2.h>

// ==============================================================================
// Namespace
// ==============================================================================

using namespace std;
using namespace XBot;

// ==============================================================================
// Class
// ==============================================================================

/**
 * @brief The CartesianImpedanceController class
 */
class CartesianImpedanceController
{

public:

    // ==============================================================================
    // Constructor and Destructor
    // ==============================================================================

    CartesianImpedanceController();

    CartesianImpedanceController(ModelInterface::Ptr model,
                                 Eigen::Matrix6d stiffness,
                                 const string end_effector,
                                 const string base_link,
                                 double damping_factor);

    //~CartesianImpedanceController();

    // ==============================================================================
    // Additional Functions
    // ==============================================================================

    /**
     * @brief compute_force is the function that given all the error of acc, vel and pos compute the virtual force of
     * the damper and spring system, using the previously updated operational space inertia, stiffness and damping matrix.
     * This function is intended to be called every iteration of a cycle, in order to obtain at each instant of time _dt
     * a new value of the virtal force generated by the displacement from the reference value of the spring/damper system
     * @return return the wrench containig the force and moment
     */
    Eigen::VectorXd compute_torque();

    // ==============================================================================
    // Setter and Getter
    // ==============================================================================

    /**
     * @brief set_reference_value sets the reference values used to compute the error in the impedance dynamic
     * @param acc_ref is the reference acceleration value of the end effector
     * @param vel_ref is the reference velocity value of the end effector
     * @param pos_ref is the reference position value of the end effector
     */
    void set_reference_value();

    /**
     * @brief reset_logger will reset the logger used to create the .mat file able to plot variable in Matlab
     * It is mandatory to reset the logger, otherwise the file will not contain the data
     */
    //void reset_logger();

private:

    // ==============================================================================
    // Variables
    // ==============================================================================

    string _root_link = "base_link";    // name of the root link, base_link by default
    string _end_effector_link = ""; // name of the end effector link, empty string by default

    XBot::ModelInterface::Ptr _model;

    // Reference velocity, position of the end-effector w.r.t. to the root link.
    Eigen::Affine3d _x_ref, _x_real;

    // Real velocity, position of the end-effector w.r.t. to the root link
    Eigen::Vector6d _xdot_real;
    Eigen::Vector6d _xdot_prec; // used for the computation of the acceleration that is done by dv/dt

    // Error variables
    Eigen::Vector6d _edot, _e;

    // Cartesian Controller
    Eigen::Matrix6d _K_omega, _D_zeta;   // diagonal matrix that represent the elementary stiffness and damping of the Cartesian axis
    Eigen::Matrix6d _K, _D; // computed stiffness and damping matrix
    double _zeta;   //ζ -> 0 for undamped behavoir and 1 for critically damped behavior

    // Generic Matrix
    Eigen::Matrix6d _op_sp_inertia; // operational space inertial matrix, usually referred to as Λ
    Eigen::MatrixXd _J; // Jacobian matrix between the root link and the end effector
    Eigen::MatrixXd _B_inv; // inertia matrix in joint space
    Eigen::Matrix6d _Q; // resulting matrix from the Cholesky decomposition of the operational space inertia, used in the computation of the damping matrix

    // SVD decomposition
    Eigen::Matrix6d _U;
    Eigen::Matrix6d _V;
    Eigen::Vector6d _S;
    Eigen::Vector6d _S_pseudo_inverse;
    double _rho = 0.001;    // offset to guarantee positive definiteness
    double _offset = 0.01;

    // Orientation error
    Eigen::Matrix3d _rotational_error;
    Eigen::Vector3d _axis;
    double _angle;

    // Force computation
    Eigen::Vector6d _force;
    Eigen::VectorXd _torque;

    // Q computation
    Eigen::Matrix6d _Phi_B;
    Eigen::Vector6d _lambda_B;
    Eigen::Matrix6d _Lambda_B_sqrt;
    Eigen::Matrix6d _Phi_B_hat;
    Eigen::Matrix6d _A_hat;
    Eigen::Matrix6d _Phi_A;
    Eigen::Matrix6d _Phi;

    // Other used matrix
    Eigen::Vector6d _diag;  // matrix sqrt computation
    Eigen::Vector6d _eigenvalues;   // check positive definiteness

    // Eigen solver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix6d> _eigen_solver;
    Eigen::JacobiSVD<Eigen::Matrix6d> _svd;

    //XBot::MatLogger2::Ptr logger;


    // ==============================================================================
    // Additional Functions
    // ==============================================================================

    /**
     * @brief Q_computation will solve a generalized eigenvalue problem. Will internally update the
     * matrix _Q and _K_omega. Refer to documentation for the algorithm explanation
     */
    Eigen::Matrix6d Q_computation(const Eigen::Matrix6d& A, const Eigen::Matrix6d& B);

    /**
     * @brief matrix_sqrt compute the square root of each elements of the matrix
     * @param matrix
     * @return the square root of the input matrix
     */
    Eigen::Matrix6d matrix_sqrt(Eigen::Matrix6d matrix);

    /**
     * @brief isPositiveDefinite will check is the matrix is positive definite, positive semidefinite or not.
     * Will print the corresponding results
     * @param matrix is the matrix to check
     */
    void isPositiveDefinite(const Eigen::Matrix6d& matrix);

    /**
     * @brief Updates the operational space inertia matrix Λ and computes the Cholesky factor Q.
     *
     * This function updates the operational space inertia matrix Λ using the formula Λ = (J * B¯¹ * J)¯¹,
     * where J is the current relative Jacobian and B¯¹ is the current inverse of the joint space inertia matrix B.
     * Its Cholesky decomposition is computed to obtain the lower triangular matrix Q. The updated matrices
     * are stored internally for further use.
     */

    void update_inertia();

    /**
     * @brief update_D compute the new value of the damping matrix after computing Q and the natural frequency matrix _K_omega
     */
    void update_D();

    /**
     * @brief update_real_value will get from the model the updated parameters of the end effector (pos, vel, acc), that are
     * used to compute the error in the impedance controller.
     */
    void update_real_value();

    /**
     * @brief compute_error compute the error between the reference and real values.
     * It must be done after getting the updated values of the acceleration, velocity, position of the
     * end-effector.
     */
    void compute_error();

    /**
     * @brief svd_inverse compute the inverse of a matrix using SVD decomposition
     *
     * In this function the input matrix is singular (so not full rank), so it is not invertible. Therefore is possible
     * to use the dirty & quick method of the inverse using the SVD
     * @param matrix that has to be inverted
     * @return the inverted matrix using the explained method
     */
    Eigen::Matrix6d  svd_inverse(Eigen::Matrix6d matrix);

    /**
     * @brief f is the function used to compute the diagonal values of the pseudo inverse of the singular values matrix
     * @param x is the singular value to be inverted
     * @return The resulting value of the function
     */
    double f(double x);

    /**
     * @brief orientation_error use the method explained at pg 139 of the Robotics Modelling, Planning and Control (Siciliano),
     * in particula the Axis and angle method.
     * @return Will return the orientation angle computed as Ref - Real
     */
    Eigen::Vector3d orientation_error();

};

#endif // CARTESIANIMPEDANCECONTROLLER_H
