#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H

// ==============================================================================
// Include
// ==============================================================================

#include <cartesianimpedancecontroller.h>
#include <cartesianimpedancesolver.h>
#include <stability_compensation.h>
#include <urdf_model/model.h>

#include <riccardo_giacchino_master_thesis/Custom_torque.h>

#include "../../src/cartesian_impedance_controller/cartesio_ros_wrapper.h"

// ==============================================================================
// Namespace
// ==============================================================================

using namespace XBot::Cartesian;

// ==============================================================================
// Class
// ==============================================================================

class ControllerManager : public ControlPlugin
{
public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void on_start() override;

    void run() override;

    void on_stop() override;

    void on_close() override;

private:

    // ==============================================================================
    // Variables
    // ==============================================================================

    std::map<std::string, ControlMode> _ctrl_map;   // map for the set the Control mode for each joint

    int stab_controller_enable;

    double _zero = 0.0; // it is the value at which we want to set the stiffness
                        // and damping to deactivate the joint impedance controller.
                        // If 0.0 the joint impedance controller is disabled, usually that's what we want, that is why is by default

    XBot::ModelInterface::Ptr _model;

    JointNameMap _stiff_initial_state, _damp_initial_state;
    JointNameMap _stiff_tmp_state, _damp_tmp_state;

    JointNameMap _motor_position;

    shared_ptr<XBot::Cartesian::Context> _ctx;

    shared_ptr<CartesianInterfaceImpl> _solver;

    double _dt, _time;

    Eigen::VectorXd _torque_cartesian;
    Eigen::VectorXd _torque_contact;
    Eigen::Vector6d wrench;
    Eigen::VectorXd _torque;

    Eigen::VectorXd _qhome;

    AggregatedTask _tasks;
    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;

    std::unique_ptr<CartesioRosWrapper> _ros_wrapper;

    // Gravity compensation variables
    Eigen::VectorXd _gravity_torque;
    Eigen::VectorXd _g;
    vector<Eigen::MatrixXd> _J_c;
    Eigen::MatrixXd _J_cz, _J_cz_pseudo_inverse;
    Eigen::VectorXd _contact_force_z; // 24 x 1

    // Computing JdotQdot and the non-linear terms
    Eigen::Vector6d _J_dot_Q_dot;
    Eigen::VectorXd _total_Jd_Qd;
    int current_index;
    Eigen::VectorXd _non_linear_torque;

    vector<Eigen::MatrixXd> _J_leg;

    // Controller of the roll and pitch angle
    map<std::shared_ptr<InteractionTask>, std::unique_ptr<StabilityCompensation>> _stability_controller;
    YAML::Node _config_parameters_stab_controller;

    // ========================== DEBUG ==========================
    XBot::MatLogger2::Ptr _logger;
    // Eigen::Affine3d pos_real, pos_ref;
    // Eigen::Matrix3d orient;
    // ImuSensor::ConstPtr _imu;
    // Eigen::VectorXd _vel;
    // bool flag = false;

    ImuSensor::ConstPtr _imu;
    Eigen::Matrix3d _floating_base_orientation;
    Eigen::Vector3d _angular_vel_imu;
    Eigen::Vector3d _linear_acc_imu;

    RosSupport::Ptr _ros;
    riccardo_giacchino_master_thesis::Custom_torque _msg;
    PublisherPtr<riccardo_giacchino_master_thesis::Custom_torque> _stats_publisher;


    // ==============================================================================
    // Additional Private Functions
    // ==============================================================================

    /**
     * @brief Retrieves tasks and casts them into InteractionTask objects.
     *
     * This function iterates through the stored tasks and attempts to cast each task
     * from the generic task type to the InteractionTask type using dynamic_pointer_cast.
     * If successful, the task is added to the list of InteractionTask objects.
     *
     */
    void get_task();

    /**
     * @brief joint_map_generator find the list of joints that are contained in a chain defined by a root link and end effector link
     *
     * It traverses the URDF model, starting from the distal link of the chain and iterates until it reaches
     * the base link or 'pelvis', assigning control modes for each encountered joint as well as initialize to
     * zero the stiffness and dammping for that joint.
     *
     */
    void joint_map_generator();

    void compute_gravity_compensation();

    void control_law();

    void stability_controller_initialization();

    void vectorEigenToMsg();

};

#endif // CONTROLLERMANAGER_H
