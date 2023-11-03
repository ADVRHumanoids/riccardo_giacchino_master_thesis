// ------------------ LIBRARIES ------------------ //
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <iostream>
#include <thread>
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>


using namespace XBot::Cartesian;


// ------------------ MAIN ------------------ //
int main(int argc, char **argv)
{


    ros::init(argc, argv, "prova_node"); // Initialization of ROS as a node named "prova_node"

    // Defining all the needed configuration parameter for the robot
    XBot::ConfigOptions xbot_cfg;

    xbot_cfg.set_urdf_path("/home/riccardo/forest_ws/ros_src/iit-centauro-ros-pkg/centauro_urdf/urdf/centauro.urdf");   //urdf file
    xbot_cfg.set_srdf_path("/home/riccardo/forest_ws/ros_src/iit-centauro-ros-pkg/centauro_srdf/srdf/centauro.srdf");   //srdf file

    xbot_cfg.generate_jidmap();

    xbot_cfg.set_parameter("is_model_floating_base", true);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
    
    auto model = XBot::ModelInterface::getModel(xbot_cfg); //Creation of the model class

    //Setting homing postion of the robot
    Eigen::VectorXd qhome;  //It is used a dynamic vector (in particular type double) whose size can be changed
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    Utils::RobotStatePublisher rspub(model); //creating ros publisher



    /*
     * Definition of a shared pointer of the type Context, that as a input parameters has another shared pointer
     * of type Parameter
     * type Context -> ?
     * type Paramenter -> ?
     *
     * A shared pointer is a type of pointer that automatically manage the dynamimcally allocated
     * object the pointer points to\
     *
    */
    const double dt = 0.01;
    auto ctx = std::make_shared<Context>(
        std::make_shared<Parameters>(dt),
        model
        );


    //Defining the inverse kinematic problem from the yaml file
    auto ik_pb_yaml = YAML::LoadFile("/home/riccardo/catkin_ws/src/package_first/src/problem_des.yaml");
    ProblemDescription ik_pb(ik_pb_yaml, ctx);


    // CartesIO solver "OpenSot". Compute the solution to the inverse kinematic problem, given a set of task (ojbect ik_pb)
    // and the definition of the model (ctx)
    // ik -> find joint variables given position and orientation
    auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_pb, ctx);


    // Obtaining from the solver the task created in the yaml file
    auto task = solver->getTask("front_lf_leg");    // The returned task i GenericTask type. It is required to cast
                                                    //it into a CartesianTask

    auto task_cartesian = std::dynamic_pointer_cast<CartesianTask>(task);   // Casting from GenericTask into a CartesianTask

    int current_state = 0;
    double time = 0;
    Eigen::VectorXd q, qdot, qddot; //joints position, velocity and acceleration, computed from the solver
    Eigen::Affine3d Tref;   //Object to hold the parameters about position and orientation

    ros::Rate rate(100);    //publishing rate on ROS topic

    while(true){
        rspub.publishTransforms(ros::Time::now(), "");
        rate.sleep();

        if(current_state == 0) // here we command a reaching motion
        {

            task_cartesian->getPoseReference(Tref); // save in Tref the current reference pose between the frame of the task and its parent of the
                                                    // given task

            Eigen::Matrix3d initial_rotation = Tref.rotation();

            std::cout << "Initial rotational matrix: " << std::endl;    //print of initial matrix
            std::cout << initial_rotation << std::endl;

            // Define a 90-degree rotation about the Z-axis of frame 2
            double angleInRadians = M_PI / 2.0;  // 90 degrees in radians
            Eigen::Matrix3d rotationZ;
            rotationZ = Eigen::AngleAxisd(angleInRadians, Eigen::Vector3d::UnitZ());

            // Evaluate the product of initialRotation and rotationY
            Tref.linear() = initial_rotation * rotationZ;

            double target_time = 5.0;   // 5 sec to do the operation
            task_cartesian->setPoseTarget(Tref, target_time);

            std::cout << std::endl;
            std::cout << std::endl;

            std::cout << "Final rotational matrix: " << std::endl;  //print of final matrix
            std::cout << Tref.rotation() << std::endl;

            current_state++;
        }

        if(current_state == 1) // here we check that the reaching started
        {
            if(task_cartesian->getTaskState() == State::Reaching)
            {
                std::cout << "Motion started!" << std::endl;
                current_state++;
            }
        }

        if(current_state == 2) // here we wait for it to be completed
        {
            if(task_cartesian->getTaskState() == State::Online)
            {
                Eigen::Affine3d T;
                task_cartesian->getCurrentPose(T);

                std::cout << "Motion completed, final error is " <<
                    (T.inverse()*Tref).translation().norm() << std::endl;

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

    }



}

