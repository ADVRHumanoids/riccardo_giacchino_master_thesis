// --------------------- LIBRARIES --------------------- //
#include <cartesian_interface/utils/RobotStatePublisher.h> // ROS related
#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <iostream> // Standard
#include <thread>   // Standar
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <chrono>
#include <motiongenerator.h>


// --------------------- NAMESPACE --------------------- //
using namespace XBot::Cartesian;
using namespace std;
using namespace Eigen;

/*
========================================================================================================
                                                MAIN
========================================================================================================
*/
int main (int argc, char **argv){

    ros::init(argc, argv, "legs_motion_node");
    ros::NodeHandle nh ("xbotcore");

    const double dt = 0.01; // sampling time

    MotionGenerator pb(nh, dt);

    string default_path = string("/home/riccardo/forest_ws/src/package_first/src/pb_description_yaml/");

    double how_much = 0.1;
    Eigen::Affine3d disp = Eigen::Affine3d::Identity(); // [0,1,2] → [x,y,z]

    pb.set_yaml_path(default_path + "pb_des_leg.yaml"); // definte yaml file path
    pb.generate_solver();   // generate the solver

//    vector<string> contact = {"contact_2",
//                              "contact_3",
//                              "contact_4"};

//    pb.set_support_polygon(contact);
//    pb.center_com();

//    pb.set_task_name("contact_wheel_4");

//    disp.translation() = Eigen::Vector3d(0, 0, how_much);
//    pb.motion(disp);    // up

//    disp.translation() = Eigen::Vector3d(0, 0, -how_much);
//    pb.motion(disp);    // down


    vector<string> contact = {"contact_1",
                              "contact_2",
                              "contact_3",
                              "contact_4"};

    vector<string> motion_sequence = {"contact_wheel_1",
                                      "contact_wheel_2",
                                      "contact_wheel_3",
                                      "contact_wheel_4"};

    int i = 0;
    for (i = 0; i < motion_sequence.size(); i++){

        // Defining the future support polygon compose just by the frame that will be in contact
        vector<string> future_con;
        for (int j = 0; j < contact.size(); j++){
            if (j == i) continue;
            else future_con.push_back(contact[j]);
        }

        // Set the new future support polygon;
        pb.set_support_polygon(future_con);

        // Move the CoM to the center of the support polygon defined previously
        pb.center_com();

        // Leg motion
        pb.set_task_name(motion_sequence[i]);

        disp.translation() = Eigen::Vector3d(0, 0, how_much);
        pb.motion(disp);    // up

        disp.translation() = Eigen::Vector3d(0, 0, -how_much);
        pb.motion(disp);    // down
    }

    return 0;

}

