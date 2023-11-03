// ------------------ LIBRARIES ------------------ //
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <iostream>
#include <thread>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>


using namespace XBot::Cartesian;


// ------------------ MAIN ------------------ //
int main(int argc, char **argv)
{


    ros::init(argc, argv, "prova_node"); // Initialization of ROS as a node named "prova_node"



    // TODO: 1 function
    // Defining all the needed configuration parameter for the robot
    ros::NodeHandle n("xbotcore");
    XBot::ConfigOptions xbot_cfg = XBot::ConfigOptionsFromParamServer(n);


    XBot::RobotInterface::Ptr robot;
    auto model = XBot::ModelInterface::getModel(xbot_cfg); //Creation of the model class

    try
    {
        robot = XBot::RobotInterface::getRobot(xbot_cfg);
        model->syncFrom(*robot);
        robot->setControlMode(XBot::ControlMode::Position());
        robot->setControlMode(
            {
             {"j_wheel_1", XBot::ControlMode::Velocity()},
             {"j_wheel_2", XBot::ControlMode::Velocity()},
             {"j_wheel_3", XBot::ControlMode::Velocity()},
             {"j_wheel_4", XBot::ControlMode::Velocity()}
            });
    }
    catch (...)
    {
        // Setting homing postion of the robot
        Eigen::VectorXd qhome;  //It is used a dynamic vector (in particular type double) whose size can be changed
        model->getRobotState("home", qhome);
        model->setJointPosition(qhome);
        model->update();
    }
    //-------------------------------------------------------------------------------------------------------------------------------



    // TODO: 2 function
    //-------------------------------------------------------------------------------------------------------------------------------
    const double dt = 0.01; //needed in the code


    // TODO: 3 function
    // Creation of the solver for the inverse kinematic
    auto ctx = std::make_shared<Context>(
        std::make_shared<Parameters>(dt),
        model
        );

    //Defining the inverse kinematic problem from the yaml file
    auto ik_pb_yaml = YAML::LoadFile("/home/riccardo/catkin_ws/src/package_first/src/pb_des_leg.yaml");
    ProblemDescription ik_pb(ik_pb_yaml, ctx);


    // CartesIO solver "OpenSot". Compute the solution to the inverse kinematic problem, given a set of task (ojbect ik_pb)
    // and the definition of the model (ctx)
    // ik -> find joint variables given position and orientation
    auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_pb, ctx);
    //-------------------------------------------------------------------------------------------------------------------------------


    // TODO: 4 function
    // Obtaining from the solver the task created in the yaml file
    // The returned task i GenericTask type. It is required to cast it into a CartesianTask
    auto task_fll = solver->getTask("front_lf_leg");
    auto task_frl = solver->getTask("front_dx_leg");
    auto task_rll = solver->getTask("rear_lf_leg");
    auto task_rrl = solver->getTask("rear_dx_leg");

    // Casting from GenericTask into a CartesianTask
    auto task_fll_cartesian = std::dynamic_pointer_cast<CartesianTask>(task_fll);
    auto task_frl_cartesian = std::dynamic_pointer_cast<CartesianTask>(task_frl);
    auto task_rll_cartesian = std::dynamic_pointer_cast<CartesianTask>(task_rll);
    auto task_rrl_cartesian = std::dynamic_pointer_cast<CartesianTask>(task_rrl);
    //-------------------------------------------------------------------------------------------------------------------------------

    int current_state = 0;
    double time = 0;
    Eigen::VectorXd q, qdot, qddot; //joints position, velocity and acceleration, computed from the solver
    Eigen::Affine3d Tref_ffl, Tref_frl, Tref_rll, Tref_rrl;   //Object to hold the parameters about position and orientation


    Utils::RobotStatePublisher rspub(model); //creating ros publisher
    ros::Rate rate(100);    //publishing rate on ROS topic

    std::cout << "Press ENTER to start the motion... \n";
    std::cin.ignore();

    while(true){
        rspub.publishTransforms(ros::Time::now(), "");
        rate.sleep();

        if(current_state == 0) // here we command a reaching motion
        {

            task_fll_cartesian->getPoseReference(Tref_ffl);
            task_frl_cartesian->getPoseReference(Tref_frl);
            task_rll_cartesian->getPoseReference(Tref_rll);
            task_rrl_cartesian->getPoseReference(Tref_rrl);


            Tref_ffl.translation()[2] += 0.2;
            //Tref_frl.translation()[2] += 0.2;
            //Tref_rll.translation()[2] += 0.2;
            //Tref_rrl.translation()[2] += 0.2;

            double target_time = 2.0;   // 5 sec to do the operation

            task_fll_cartesian->setPoseTarget(Tref_ffl, target_time);
            task_rll_cartesian->setPoseTarget(Tref_rll, target_time);
            task_frl_cartesian->setPoseTarget(Tref_frl, target_time);
            task_rrl_cartesian->setPoseTarget(Tref_rrl, target_time);

            current_state++;
        }

        if(current_state == 1) // here we check that the reaching started
        {
            if(task_fll_cartesian->getTaskState() == State::Reaching
                && task_frl_cartesian->getTaskState() == State::Reaching
                && task_rll_cartesian->getTaskState() == State::Reaching
                && task_rrl_cartesian->getTaskState() == State::Reaching)
            {
                std::cout << "Motion started!" << std::endl;
                current_state++;
            }
        }

        if(current_state == 2) // here we wait for it to be completed
        {
            if(task_fll_cartesian->getTaskState() == State::Online
                && task_frl_cartesian->getTaskState() == State::Online
                && task_rll_cartesian->getTaskState() == State::Online
                && task_rrl_cartesian->getTaskState() == State::Online)
            {

                current_state++;
            }
        }

        if(current_state == 3) // here we wait the robot to come to a stop
        {
            std::cout << "qdot norm is " << qdot.norm() << std::endl;
            if(qdot.norm() < 1e-3)
            {
                std::cout << "Robot came to a stop, press ENTER to exit.. \n";
                std::cin.ignore();
                current_state++;
            }

        }

        if(current_state == 4) break;

        solver->update(time, dt);

        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        model->getJointAcceleration(qddot);

        q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
        qdot += dt * qddot;

        //std::cout << q << std::endl << "------------------------------------------------------------------------\n";

        model->setJointPosition(q);
        model->setJointVelocity(qdot);
        model->update();

        time += dt;

        if(robot)
        {
            robot->setReferenceFrom(*model);
            robot->move();
        }

    }
}
