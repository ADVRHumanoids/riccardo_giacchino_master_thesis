#include "controllermanager.h"

bool ControllerManager::on_initialize()
{
    _dt = getPeriodSec();

    _robot->sense();

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);

    _model->update();

    auto ik_pb_yaml = YAML::LoadFile(getParamOrThrow<string>("~stack_path"));

    _ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<Parameters>(_dt),  //WARNING: handle _dt
                                     _model);

    ProblemDescription pb(ik_pb_yaml, _ctx);

    _solver = CartesianInterfaceImpl::MakeInstance("ImpSolver",
                                                   pb,
                                                   _ctx);

    _tasks = pb.getTask(pb.getNumTasks());

    get_task_names();

    joint_map_generator();

    setDefaultControlMode(_ctrl_map);

    // Initialize usefull variable
    _q = _qdot = Eigen::VectorXd::Zero(_model->getJointNum());
    _torque = Eigen::VectorXd::Zero(_model->getJointNum());

    return true;

}

void ControllerManager::on_start()
{

    _robot->sense();
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    _model->update();

    _robot->getStiffness(_stiff_initial_state);
    _robot->getDamping(_damp_initial_state);

    if (!_robot->setStiffness(_stiff_tmp_state) || !_robot->setDamping(_damp_tmp_state))
        cout << "[ERROR]: unable to set stiffness or damping value" << endl;

    _robot->move();

    // Reset of the tmp stiffness and damping
    for (auto joint : joint_names){
        _stiff_tmp_state[joint] = _zero;
        _damp_tmp_state[joint] = _zero;
    }

    cout << "[INFO]: Cartesian impedance control is starting!" << endl;

    _time = 0;
    _solver->reset(_time);

}

void ControllerManager::run()
{

    _robot->sense();
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);    

    // CartesIO pluing update
    _solver->update(_time, _dt);

    _model->getJointEffort(_torque);

    _robot->setEffortReference(_torque.tail(_robot->getJointNum()));

    _model->update();

    // get effort from model
    // set effort reference -> on _robot


    // keep updated the position reference
    _model->getMotorPosition(_motor_position);
    _robot->setPositionReference(_motor_position);

    // Disable joint impedance controller
    _robot->setStiffness(_stiff_tmp_state);
    _robot->setDamping(_damp_tmp_state);

    _robot->move();

    // time update
    _time += _dt;

}

void ControllerManager::on_stop()
{

    // TODO: create ramping transition for stiffness and damping
    _robot->setStiffness(_stiff_tmp_state);
    _robot->setDamping(_damp_tmp_state);

    _robot->move();

    cout << "[INFO]: Cartesian impedance control is stopping!" << endl;
}

void ControllerManager::get_task_names(){

    // Get all the task name
    for (auto task : _tasks){

        auto tmp_task = std::dynamic_pointer_cast<InteractionTask>(task);

        if (tmp_task == nullptr)
            cerr << "[ERROR]: the cast result in a null pointer" << endl;
        else
            _tasks_casted.push_back(tmp_task);   // cast into Interaction task

    }

}

void ControllerManager::joint_map_generator(){

    // Construct joint map to set the stiffness and damping
    // TODO: test correct working
    auto urdf_model = _model->getUrdf();

    for (auto task : _tasks_casted) {

        string end_link = task->getDistalLink();

        while (end_link != task->getBaseLink()) {

            auto link = urdf_model.getLink(end_link);
            auto parent_joint = link->parent_joint;

            if (parent_joint == nullptr) {
                cerr << "[ERROR]: null pointer to parent joint on link " << end_link << endl;

            } else if (!_robot->hasJoint(parent_joint->name)) {
                cerr << "[ERROR]: robot does not have joint " << parent_joint->name << endl;

            } else {
                joint_names.push_back(parent_joint->name);
                _ctrl_map[parent_joint->name] = ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping() + ControlMode::Position();
                _stiff_tmp_state[parent_joint->name] = _zero;
                _damp_tmp_state[parent_joint->name] = _zero;
            }

            end_link = link->getParent()->name;

        }

    }

}

XBOT2_REGISTER_PLUGIN(ControllerManager, controllermanager)

