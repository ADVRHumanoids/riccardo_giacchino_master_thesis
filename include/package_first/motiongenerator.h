#ifndef MOTIONGENERATOR_H
#define MOTIONGENERATOR_H

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
#include <matlogger2/matlogger2.h>

using namespace XBot::Cartesian;
using namespace std;



class MotionGenerator
{
public:

    MotionGenerator(ros::NodeHandle nh, double dt);
    ~MotionGenerator();

    // ------- Proper Function ------- //

    /**
     * @brief motion is a function that given a certain displacement, it solver the inverse kinematic problem
     * to reach the new reference pose, computing the position and velocity of each joint in order to complete the task.
     * The motion is done by changing the reference transformation matrix between two frames associated to a particular
     * task, for a Cartesian Task.
     * @param displacement contain the displacement w.r.t. the original value of the transformation matrix. It can be
     * a rotational displacement or a translational displacement, as well as both. It depend how you set this variable.
     * This variable is added to the Affine3d object obtain from the function task->getPoseReferenct().
     */
    void motion(Eigen::Affine3d displacement);

    /**
     * @brief motion is different from the previous function since in this case the imput paramenter is not an dispacement
     * w.r.t. the actual refetence pose, but is the new reference pose of a given task. It is used to set the new pose of
     * the CoM (that fyi is the defined w.r.t. the world frame)
     * @param new_reference_pose is the actual new reference pose
     * @param flag
     */
    void motion(Eigen::Affine3d new_reference_pose, bool flag);

    /**
     * @brief center_com compute the position vector of the center of the future support polygon, than the CoM is moved to
     * the new computed position of the center of the support polygon
     */
    void center_com();

    /**
     * @brief generate_solver generate the instance of the solver based on the model and the problem description
     * The default solver is OpenSot.
     */
    void generate_solver();

    // ------- Setter ------- //

    /**
     * @brief set_yaml_path set the path of the yaml file that contain the description of the problem
     * @param path is the path to the file
     */
    void set_yaml_path(const string path);

    /**
     * @brief set_task_name set the name of the task from which we want to obtain the reference pose
     * @param task_name
     */
    void set_task_name(const string task_name);

    /**
     * @brief set_task_state is used to enable or disable particular task specified in the problem description file
     * @param task_name is the name of the task for which a status change is desired
     * @param state can be ActivationState::Enabled or ActivationState::Disabled
     */
    void set_task_state(const string task_name, ActivationState state);

    void set_support_polygon(vector<string> contact_link);


private:

    ros::NodeHandle _nh;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    string _yaml_file_path;
    string _task_name;

    shared_ptr<Context> _ctx;

    shared_ptr<CartesianInterfaceImpl> _solver;

    shared_ptr<CartesianTask> _cartesian_task;
    shared_ptr<ComTask> _com_task;

    vector<Eigen::Vector3d> _support_polygon;
    Eigen::Vector3d _center;

    XBot::MatLogger2::Ptr logger;

    double _dt;
    double _time;

    void cartesian_task();

    void com_task();

    void compute_polygon_center();

};

#endif // MOTIONGENERATOR_H
