#include "controllermanager.h"

bool ControllerManager::on_initialize()
{
    _dt = getPeriodSec();

    _robot->sense();

    // Model creation
    _model = ModelInterface::getModel(_robot->getConfigOptions());
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    _model->update();

    // Take the path to the problem definition file
    auto ik_pb_yaml = YAML::LoadFile(getParamOrThrow<string>("~stack_path"));

    _ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<Parameters>(_dt),  //WARNING: handle _dt
                                     _model);
    ProblemDescription pb(ik_pb_yaml, _ctx);

    // Solver creation
    _solver = CartesianInterfaceImpl::MakeInstance("ImpSolver",
                                                   pb,
                                                   _ctx);

    // Obtaining the tasks from the problem description
    _tasks = pb.getTask(pb.getNumTasks()-1);

    // Extract task names
    get_task();

    // Create the joint map
    joint_map_generator();

    // Set control mode for the joints found in the previous function
    setDefaultControlMode(_ctrl_map);

    // Initialize usefull variable
    _torque = Eigen::VectorXd::Zero(_model->getJointNum());

    return true;

}

void ControllerManager::on_start()
{

    // Robot and Model update
    _robot->sense();
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    _model->update();

    // Memorize the initial joint stiffness and damping
    _robot->getStiffness(_stiff_initial_state);
    _robot->getDamping(_damp_initial_state);

    // Set joint stiffness and damping to zero in order to disable joint impedance controller
    if (!_robot->setStiffness(_stiff_tmp_state) || !_robot->setDamping(_damp_tmp_state))
        cout << "[ERROR]: unable to set stiffness or damping value" << endl;

    _robot->move();

    _time = 0;
    _solver->reset(_time);

    cout << "[INFO]: Cartesian impedance control is starting!" << endl;
}

void ControllerManager::run()
{

    _robot->sense();
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);    

    // CartesIO pluing update will compute the torque base on the controller and set them into the model
    _solver->update(_time, _dt);

    // Get the torque from the model
    _model->getJointEffort(_torque);

    // Set them in the robot
    _robot->setEffortReference(_torque.tail(_robot->getJointNum()));

    _model->update();

    // keep updated the position reference
//    _model->getMotorPosition(_motor_position);
//    _robot->setPositionReference(_motor_position);

    // Keep disabled joint impedance controller
    _robot->setStiffness(_stiff_tmp_state);
    _robot->setDamping(_damp_tmp_state);

    _robot->move();

    // Update the time for the solver
    _time += _dt;

}

void ControllerManager::on_stop()
{

    // Reset the joint stiffness and damping to initial values in order to enable joint impedance controller
    _robot->setStiffness(_stiff_initial_state);
    _robot->setDamping(_damp_initial_state);

    _robot->move();

    cout << "[INFO]: Cartesian impedance control is stopping!" << endl;
}

void ControllerManager::get_task(){

    for (auto task : _tasks){

        // Cast from Generic task to Interaction task
        auto tmp_task = std::dynamic_pointer_cast<InteractionTask>(task);

        // Check null pointer
        if (tmp_task == nullptr)
            cerr << "[ERROR]: the cast result in a null pointer" << endl;
        else
            _tasks_casted.push_back(tmp_task);   // cast into Interaction task

    }

}

void ControllerManager::joint_map_generator(){

    auto urdf_model = _model->getUrdf();

    for (auto task : _tasks_casted) {

        string end_link = task->getDistalLink();

        while (end_link != task->getBaseLink() && end_link != "pelvis") {

            auto link = urdf_model.getLink(end_link);

            auto parent_joint = link->parent_joint;

            if (parent_joint == nullptr) {
                cerr << "[ERROR]: null pointer to parent joint on link " << end_link << endl;

            } else if (!_robot->hasJoint(parent_joint->name)) {
                cerr << "[WARNING]: robot does not have joint " << parent_joint->name << endl;

            } else {
                _ctrl_map[parent_joint->name] = ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping() + ControlMode::Position();
                _stiff_tmp_state[parent_joint->name] = _zero;
                _damp_tmp_state[parent_joint->name] = _zero;
            }

            end_link = link->getParent()->name;

        }

    }

}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)

