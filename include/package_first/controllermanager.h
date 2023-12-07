#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H

// ==============================================================================
// Include
// ==============================================================================

#include <cartesianimpedancecontroller.h>
#include <cartesianimpedancesolver.h>
#include <urdf_model/model.h>

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

    Eigen::VectorXd _torque;

    Eigen::VectorXd _qhome;

    AggregatedTask _tasks;
    std::vector<std::shared_ptr<InteractionTask>> _tasks_casted;

    std::unique_ptr<CartesioRosWrapper> _ros_wrapper;

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

};

#endif // CONTROLLERMANAGER_H
