#include <cartesian_interface/utils/RobotStatePublisher.h>  // ROS related
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
#include <xbot_msgs/JointCommand.h>


using namespace XBot;
using namespace XBot::Cartesian;
using namespace std;
using namespace Eigen;



/**
 * @brief The VelocityController class should create a PI Controller to control the velocity of the robot while limit the increment in the
 * velocity to a max of 0.1 (increment) at a time
 */
class VelocityController {

public:

    VelocityController(double target_velocity, double dt)
        : _target_velocity(target_velocity),
        _integral_error(0.0),
        _proportional_gain(1.0),  // Adjust this gain to control acceleration
        _time_step(dt) {}

    double update(double current_velocity) {
        double error = _target_velocity - current_velocity;
        _integral_error += error * _time_step;
        double control_output = _proportional_gain * error + _integral_error;

        // Limit the control output to the desired acceleration
        if (control_output > 0.1) {
            control_output = 0.1;
        } else if (control_output < -0.1) {
            control_output = -0.1;
        }

        return current_velocity + control_output * _time_step;
    }

private:

    double _target_velocity;
    double _integral_error;
    double _proportional_gain;
    double _time_step;
};


int main (int argc, char **argv){

    ros::init(argc, argv, "vel_controller");

    ros::NodeHandle nh("xbotcore");

    auto pub_vel = nh.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);

    float vel = 5;
    float ang_vel = 0;
    float r = 0.124;    // [m]

    std::vector<std::basic_string<char>> names =
    {  // name
        "j_wheel_1",
        "j_wheel_2",
        "j_wheel_3",
        "j_wheel_4"
    };

    xbot_msgs::JointCommand msg;
    msg.name = names;

    // Get the bitset representation
    const std::vector<unsigned char> my_vector = {0b00010, 0b00010, 0b00010, 0b00010};
    msg.ctrl_mode = my_vector;


    ros::Rate rate(10);

    std::cout << "Press ENTER to start the motion... \n";
    std::cin.ignore();

    while(true){

        rate.sleep();

        ang_vel = vel/r;    // [rad/s]

        std::vector<float> velocities = {
            ang_vel,
            - ang_vel,
            ang_vel,
            - ang_vel
        };

        msg.velocity = velocities;
        pub_vel.publish(msg);

    }

}
