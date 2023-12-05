// ------------------ LIBRARIES ------------------ //
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <iostream>
#include <thread>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <motiongenerator.h>

using namespace XBot::Cartesian;

//WARNING: Not working
//Excluded in the CMakeFile.txt

int main(int argc, char **argv)
{

    //ROS initialization
    ros::init(argc, argv, "base_motion_node");

    //Robot parameter configuration
    XBot::RobotInterface::Ptr robot;
    XBot::ModelInterface::Ptr model;
    mg.model_generation(robot, model);

    const double dt = 0.01; //integration time

    //Solver creation
    CartesianInterfaceImpl::Ptr solver;
    mg.solver_creation(model, solver, dt);

    //Task extraction
    CartesianTask::Ptr task_base = mg.get_task_cartesian("base_motion", solver);

    //Variable definition for the integratin
    int current_state = 0;
    double time = 0;
    Eigen::VectorXd q, qdot, qddot; //joints position, velocity and acceleration, computed from the solver
    Eigen::Affine3d Tref_base;

    //Publishing the model of the robot
    Utils::RobotStatePublisher rspub(model); //creating ros publisher
    ros::Rate rate(100);    //publishing rate on ROS topic, frequency in Hz

    std::cout << "Press ENTER to start the motion... \n";
    std::cin.ignore();

    while(true){

        rspub.publishTransforms(ros::Time::now(), "");
        rate.sleep();

        if(current_state == 0){
            task_base->getPoseReference(Tref_base);

            Tref_base = mg.translation(2, -0.1, Tref_base);   //translation

            double target_time = 5.0;   // 5 sec to do the operation
            task_base->setPoseTarget(Tref_base, target_time);

            current_state++;

        }


        if(current_state == 1) // here we check that the reaching started
        {
            if(task_base->getTaskState() == State::Reaching)
            {
                std::cout << "Motion started!" << std::endl;
                current_state++;
            }
        }

        if(current_state == 2) // here we wait for it to be completed
        {
            if(task_base->getTaskState() == State::Online)
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
