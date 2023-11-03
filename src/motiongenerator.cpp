// ------------------ LIBRARIES ------------------ //
#include <motiongenerator.h>

using namespace XBot::Cartesian;
using namespace std;

// Public

MotionGenerator::MotionGenerator(ros::NodeHandle nh,
                                 double dt):

    _nh(nh),
    _dt(dt)
{

    XBot::ConfigOptions xbot_cfg = XBot::ConfigOptionsFromParamServer(_nh);
    _model = XBot::ModelInterface::getModel(xbot_cfg);

    try
    {
        _robot = XBot::RobotInterface::getRobot(xbot_cfg);
        _model->syncFrom(*_robot);
        _robot->setControlMode(XBot::ControlMode::Position());
        _robot->setControlMode(
            {
                {"j_wheel_1", XBot::ControlMode::Velocity()},
                {"j_wheel_2", XBot::ControlMode::Velocity()},
                {"j_wheel_3", XBot::ControlMode::Velocity()},
                {"j_wheel_4", XBot::ControlMode::Velocity()}
            });
    }
    catch (const std::exception& e)
    {
        // Setting homing postion of the robot
        Eigen::VectorXd qhome;
        _model->getRobotState("home", qhome);
        _model->setJointPosition(qhome);
        _model->update();
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);
        cerr << "[ Error ]: " << e.what() << endl;
    }

    _ctx = std::make_shared<Context>(std::make_shared<Parameters>(_dt),
                                     _model);

    _yaml_file_path = "";
    _task_name = "";
    _time = 0;
    logger = XBot::MatLogger2::MakeLogger("/tmp/logger.mat");

}

// Destructor
MotionGenerator::~MotionGenerator(){
    cout << "[ NOTE ]: Exiting" << endl;
}

void MotionGenerator::set_yaml_path(const string path){

    _yaml_file_path = path;
    //TODO: create a way to check the path before say OK
    cout << "\n[ OK ] Set path: " << _yaml_file_path << endl;

}


void MotionGenerator::set_task_name(const string task_name){

    _task_name = task_name;
    cout << "[ OK ] Set task name: " << _task_name << endl;

}

void MotionGenerator::set_support_polygon(vector<string> contact_link){

    _support_polygon = {};
    for (const string& con : contact_link){
        Eigen::Affine3d con_wrt_world;
        _model->getPose(con, con_wrt_world);
        _support_polygon.push_back(con_wrt_world.translation());
    }

}

void MotionGenerator::set_task_state(const string task_name, ActivationState state){

    _solver->setActivationState(task_name, state);

}

void MotionGenerator::generate_solver(){

    auto ik_pb_yaml = YAML::LoadFile(_yaml_file_path);

    ProblemDescription ik_pb(ik_pb_yaml, _ctx);

    _solver = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_pb, _ctx);

}

void MotionGenerator::motion(Eigen::Affine3d displacement){

    int current_state = 0;
    double motion = 0;

    Eigen::Affine3d Tref;   // transformation matrix
    Eigen::VectorXd q, qdot, qddot; // joint position (q), velocity (qdot), acceleration (qddot)

    try {
        // Check if the string variable containing the yaml file and task name are empty
        // They should already be set before calling this function
        if(_yaml_file_path.empty())
            throw std::runtime_error("_yaml_file_path is an empty string. Use set_yaml_path() to set a path");

        if(_task_name.empty())
            throw std::runtime_error("_task_name is an empty string. Use set_task_name() to specify the task");

    } catch (const std::exception& e) {

        std::cerr << "[ Error ]: " << e.what() << endl;

    }

    cartesian_task();

    const std::chrono::duration<double> loop_period(_dt);  // period of the loop

    while (true) {  // while loop until it ends the motion

        auto start_time = std::chrono::high_resolution_clock::now();

        if(current_state == 0)  // set the new reference pose
        {
            _cartesian_task->getPoseReference(Tref);

            string msg = string("[ OK ] Moving ") + _task_name;
            cout << msg << endl;

            Tref.translation() += displacement.translation();
            //cout << Tref.translation() << endl;

            double target_time = 2.0;

            _cartesian_task->setPoseTarget(Tref, target_time);

            current_state++;
        }

        if(current_state == 1)  // checking the starting of the motion
            if(_cartesian_task ->getTaskState() == State::Reaching){
                current_state++;
                cout << "[ OK ] Motion started!" << endl;
            }
        if(current_state == 2)  // checking the correct execution of the motion
            if(_cartesian_task ->getTaskState() == State::Online)
                current_state++;

        if(current_state == 3)  // checking the end of the motion, when the joint velocity are really close to zero
            // since reaching the new reference pose means stop moving
            if(qdot.norm() < 1e-3){
                current_state++;
                cout << "[ OK ] Motion executed!" << endl;
            }

        if(current_state == 4)  //exiting from while loop
            break;


        if (!_solver->update(_time, _dt))
        {
            ROS_WARN("unable to solve!");
        }

        // Get model variables
        _model->getJointPosition(q);
        _model->getJointVelocity(qdot);

        // Compute the new values
        q += _dt * qdot;

        // Set and update the model with the new value
        _model->setJointPosition(q);
        _model->update();

        //update the time
        _time += _dt;

        // update the robot to make it moves in the simulation (Gazebo)
        if(_robot)
        {
            _robot->setReferenceFrom(*_model);
            _robot->move();
        }
        else
        {
            _rspub->publishTransforms(ros::Time::now(), "ci");
        }

        // Compute the time used to execute the previous part of the loop
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

        // If the remaining time is smaller than the loop period, then wait to reach the loop period,
        // and then proceed with the next iteration of the cycle, otherwise proceed
        if (elapsed_time < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed_time);
        }

    }

}


void MotionGenerator::motion(Eigen::Affine3d new_reference_pose, bool flag){

    int current_state = 0;
    double motion = 0;

    Eigen::Affine3d Tref;   // transformation matrix
    Eigen::VectorXd q, qdot, qddot; // joint position (q), velocity (qdot), acceleration (qddot)

    try {
        // Check if the string variable containing the yaml file and task name are empty
        // They should already be set before calling this function
        if(_yaml_file_path.empty())
            throw std::runtime_error("_yaml_file_path is an empty string. Use set_yaml_path() to set a path");

        if(_task_name.empty())
            throw std::runtime_error("_task_name is an empty string. Use set_task_name() to specify the task");

    } catch (const std::exception& e) {

        std::cerr << "[ Error ]: " << e.what() << endl;

    }

    com_task();

//    // DEBUG
//    logger->create("position", 46, 1, 1e6);
//    logger->create("velocity", 46, 1, 1e6);
//    logger->create("acceleration", 46, 1, 1e6);

    const std::chrono::duration<double> loop_period(_dt);  // period of the loop

    while (true) {  // while loop until it ends the motion

        auto start_time = std::chrono::high_resolution_clock::now();

        if(current_state == 0)  // set the new reference pose
        {

            string msg = string("[ OK ] Moving ") + _task_name;
            cout << msg << endl;

            _com_task->setPoseTarget(new_reference_pose, 4.0);

            current_state++;
        }

        if(current_state == 1)  // checking the starting of the motion
            if(_com_task ->getTaskState() == State::Reaching){
                current_state++;
                cout << "[ OK ] Motion started!" << endl;
            }
        if(current_state == 2)  // checking the correct execution of the motion
            if(_com_task ->getTaskState() == State::Online)
                current_state++;

        if(current_state == 3)  // checking the end of the motion, when the joint velocity are really close to zero
            // since reaching the new reference pose means stop moving
            if(qdot.norm() < 1e-3){
                current_state++;
                cout << "[ OK ] Motion executed!" << endl;
            }

        if(current_state == 4){  //exiting from while loop
            break;
        }

        if (!_solver->update(_time, _dt))
        {
            ROS_WARN("unable to solve!");
        }

        // Get model variables
        _model->getJointPosition(q);
        _model->getJointVelocity(qdot);

        // Compute the new values
        q += _dt * qdot;

        //DEBUG
//        logger->add("position", q);
//        logger->add("velocity", qdot);
//        logger->add("acceleration", qddot);

        // Set and update the model with the new value
        _model->setJointPosition(q);
        _model->update();

        //update the time
        _time += _dt;

        // update the robot to make it moves in the simulation (Gazebo)
        if(_robot)
        {
            _robot->setReferenceFrom(*_model);
            _robot->move();
        }
        else
        {
            _rspub->publishTransforms(ros::Time::now(), "ci");
        }

        // Compute the time used to execute the previous part of the loop
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

        // If the remaining time is smaller than the loop period, then wait to reach the loop period,
        // and then proceed with the next iteration of the cycle, otherwise proceed
        if (elapsed_time < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed_time);
        }

    }

}


void MotionGenerator::center_com(){

    // Compute the center of the actual support polygon
    compute_polygon_center();

    // Now the variable _center will contain the position vector w.r.t. the world frame
    // that is the new reference position that we want the CoM to have

    set_task_name("com_position");

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = _center;

    motion(pose, true);

}

// Private

void MotionGenerator::cartesian_task(){

    _cartesian_task = std::dynamic_pointer_cast<CartesianTask>(_solver->getTask(_task_name));

}


void MotionGenerator::com_task(){

    _com_task = std::dynamic_pointer_cast<ComTask>(_solver->getTask(_task_name));

}


void MotionGenerator::compute_polygon_center(){

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();

    int counter = 0;

    for (Eigen::Vector3d point : _support_polygon) {
        sum.x() += point.x();
        sum.y() += point.y();
        counter++;
    }

    _center = sum/counter;
    cout << "[ INFO ]: Computed center: \n" << _center << endl;

}





